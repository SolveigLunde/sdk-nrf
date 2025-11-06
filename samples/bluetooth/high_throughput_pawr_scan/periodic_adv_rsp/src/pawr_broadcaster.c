#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <string.h>

#include "pawr_broadcaster.h"
#include "pawr_onboarding.h"
#include "onboarding_manager.h"

#define NUM_SUBEVENTS 1
#define PACKET_SIZE   251

#define MAX_INDIVIDUAL_RESPONSE_SIZE 247
#define THROUGHPUT_PRINT_INTERVAL 1000
#define SLOT_IDLE_THRESHOLD 4

#define ADV_NAME "PAwR adv sample"

static struct bt_le_ext_adv *adv_handle;

static struct bt_le_per_adv_param per_adv_params;
static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

#define BITMAP_BYTES_NEEDED(num_devices) (((num_devices) + 7) / 8)

static uint8_t expected_responses[BITMAP_BYTES_NEEDED(PAWR_MAX_RESPONSE_SLOTS)];
static uint8_t response_bitmap[BITMAP_BYTES_NEEDED(PAWR_MAX_RESPONSE_SLOTS)];

static uint16_t g_num_rsp_slots_print;
static uint16_t g_payload_size_print;
static uint32_t g_adv_event_ms_x100_print;

static uint32_t total_interval_bytes;
static uint64_t throughput_timestamp;

static uint8_t noresp_counters[PAWR_MAX_RESPONSE_SLOTS];

static inline void bitmap_set_bit(uint8_t *bitmap, int bit)
{
	bitmap[bit / 8] |= (1U << (bit % 8));
}

static inline void bitmap_clear_bit(uint8_t *bitmap, int bit)
{
	bitmap[bit / 8] &= ~(1U << (bit % 8));
}

static inline bool bitmap_test_bit(const uint8_t *bitmap, int bit)
{
	return (bitmap[bit / 8] & (1U << (bit % 8))) != 0U;
}

static inline void bitmap_clear_all(uint8_t *bitmap, size_t num_bits)
{
	memset(bitmap, 0, BITMAP_BYTES_NEEDED(num_bits));
}

static void set_pawr_params(struct bt_le_per_adv_param *params, uint8_t num_response_slots)
{
	const uint16_t MIN_PAWR_INTERVAL_MS = 1;
	const float ADVERTISER_GUARD_MS = 1.0f;
	const uint8_t PHY_RATE_MBPS = 2;
	const uint16_t packet_size = MAX_INDIVIDUAL_RESPONSE_SIZE;

	const float tx_time_ms = (float)(packet_size * 8) / (PHY_RATE_MBPS * 1000.0f);
	const float slot_time_ms = tx_time_ms + 0.25f;
	uint16_t slot_spacing_units = (uint16_t)ceilf(slot_time_ms / 0.125f);
	const uint8_t delay_units = 3;

	const uint32_t subevent_duration_ms_x100 = (uint32_t)delay_units * 125U +
		(uint32_t)num_response_slots * ((uint32_t)slot_spacing_units * 125U / 10U);

	uint16_t subevent_interval_units = (uint16_t)((subevent_duration_ms_x100 + 125U - 1U) / 125U);
	if (subevent_interval_units < 6) {
		subevent_interval_units = 6;
	} else if (subevent_interval_units > 255) {
		subevent_interval_units = 255;
		printk("Warning: subevent too long for one window; consider fewer devices or smaller payload.\n");
	}

	const uint32_t total_event_time_ms_x100 = (uint32_t)delay_units * 125U +
		(uint32_t)num_response_slots * ((uint32_t)slot_spacing_units * 125U / 10U) +
		(uint32_t)(ADVERTISER_GUARD_MS * 100.0f);

	const uint16_t min_interval_units = (uint16_t)((MIN_PAWR_INTERVAL_MS + 1) / 1.25f);
	const uint16_t required_interval_units = (uint16_t)((total_event_time_ms_x100 + 125U - 1U) / 125U);
	uint16_t advertising_event_interval_units = (required_interval_units > min_interval_units)
		? required_interval_units
		: min_interval_units;

	if (advertising_event_interval_units < subevent_interval_units) {
		advertising_event_interval_units = subevent_interval_units;
	}

	params->interval_min = advertising_event_interval_units;
	params->interval_max = advertising_event_interval_units;
	params->options = 0;
	params->num_subevents = 1;
	params->subevent_interval = (uint8_t)subevent_interval_units;
	params->response_slot_delay = delay_units;
	params->response_slot_spacing = (uint8_t)slot_spacing_units;
	params->num_response_slots = num_response_slots;

	const uint32_t adv_event_ms_x100 = (uint32_t)advertising_event_interval_units * 125U;
	const uint32_t subevent_ms_x100 = (uint32_t)subevent_interval_units * 125U;
	const uint32_t slot_ms_x100 = (uint32_t)slot_spacing_units * 125U / 10U;
	const uint32_t est_throughput_kbps = (adv_event_ms_x100 > 0U) ?
		(uint32_t)(((uint64_t)num_response_slots * packet_size * 8ULL * 100000ULL) /
			 (uint64_t)adv_event_ms_x100) /
		1000U : 0U;

	g_num_rsp_slots_print = num_response_slots;
	g_payload_size_print = packet_size;
	g_adv_event_ms_x100_print = adv_event_ms_x100;

	printk("PAwR config (1 subevent):\n");
	printk("  Devices: %u, Resp size: %u B, Slot: %u.%02u ms\n",
		num_response_slots, packet_size, slot_ms_x100 / 100U, slot_ms_x100 % 100U);
	printk("  Delay: %u (%u.%02u ms), SubEvt: %u (%u.%02u ms), AdvInt: %u (%u.%02u ms)\n",
		delay_units, (uint32_t)delay_units * 125U / 100U, (uint32_t)delay_units * 125U % 100U,
		subevent_interval_units, subevent_ms_x100 / 100U, subevent_ms_x100 % 100U,
		advertising_event_interval_units, adv_event_ms_x100 / 100U, adv_event_ms_x100 % 100U);
	printk("  Est. uplink throughput (theoretical) ~%u kbps\n", (unsigned int)est_throughput_kbps);
}

static void init_bufs(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(backing_store); i++) {
		net_buf_simple_init_with_data(&bufs[i], &backing_store[i],
				ARRAY_SIZE(backing_store[i]));
		net_buf_simple_reset(&bufs[i]);
		/* Add manufacturer specific data header */
		net_buf_simple_add_u8(&bufs[i], 3);
		net_buf_simple_add_u8(&bufs[i], BT_DATA_MANUFACTURER_DATA);
		net_buf_simple_add_le16(&bufs[i], 0x0059);
	}
}

static void request_cb(struct bt_le_ext_adv *adv,
	const struct bt_le_per_adv_data_request *request)
{
	uint8_t bitmap_bytes = BITMAP_BYTES_NEEDED(PAWR_MAX_RESPONSE_SLOTS);
	uint8_t retransmit_bitmap[BITMAP_BYTES_NEEDED(PAWR_MAX_RESPONSE_SLOTS)];

	if (!request) {
		printk("Error: NULL request received\n");
		return;
	}

	uint8_t to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

	for (int i = 0; i < bitmap_bytes; i++) {
		retransmit_bitmap[i] = expected_responses[i] & (~response_bitmap[i]);
	}

	for (uint8_t slot = 0; slot < PAWR_MAX_RESPONSE_SLOTS; slot++) {
		bool expected = bitmap_test_bit(expected_responses, slot);
		bool got_response = bitmap_test_bit(response_bitmap, slot);
		if (expected && !got_response) {
			if (noresp_counters[slot] < 0xFF) {
				noresp_counters[slot]++;
			}
			if (noresp_counters[slot] >= SLOT_IDLE_THRESHOLD) {
				printk("Slot %u inactive, releasing\n", slot);
				bitmap_clear_bit(expected_responses, slot);
				onboarding_manager_handle_slot_idle(slot);
				noresp_counters[slot] = 0U;
			}
		} else if (got_response) {
			noresp_counters[slot] = 0U;
		}
	}

	bitmap_clear_all(response_bitmap, PAWR_MAX_RESPONSE_SLOTS);

	for (size_t i = 0; i < to_send; i++) {
		struct net_buf_simple *buf = &bufs[i];
		net_buf_simple_reset(buf);

		uint8_t *length_field = net_buf_simple_add(buf, 1);
		net_buf_simple_add_u8(buf, BT_DATA_MANUFACTURER_DATA);
		net_buf_simple_add_le16(buf, 0x0059);
		net_buf_simple_add_u8(buf, 0x01); /* version */
		net_buf_simple_add_u8(buf, PAWR_MAX_RESPONSE_SLOTS);

		net_buf_simple_add_u8(buf, bitmap_bytes);
		for (int j = 0; j < bitmap_bytes; j++) {
			net_buf_simple_add_u8(buf, expected_responses[j]);
		}

		net_buf_simple_add_u8(buf, bitmap_bytes);
		for (int j = 0; j < bitmap_bytes; j++) {
			net_buf_simple_add_u8(buf, retransmit_bitmap[j]);
		}

		*length_field = buf->len - 1U;

		subevent_data_params[i].subevent = request->start + i;
		subevent_data_params[i].response_slot_start = 0;
		subevent_data_params[i].response_slot_count = PAWR_MAX_RESPONSE_SLOTS;
		subevent_data_params[i].data = buf;
	}

	int err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
	if (err) {
		printk("Failed to set subevent data (err %d)\n", err);
	}
}

static void response_cb(struct bt_le_ext_adv *adv,
	struct bt_le_per_adv_response_info *info,
	struct net_buf_simple *buf)
{
	ARG_UNUSED(adv);

	if (!buf) {
		printk("Empty response at slot %u\n", info->response_slot);
		return;
	}

	if (total_interval_bytes == 0U) {
		throughput_timestamp = k_uptime_get_32();
	}

	total_interval_bytes += buf->len;

	if (info->response_slot < PAWR_MAX_RESPONSE_SLOTS) {
		bitmap_set_bit(response_bitmap, info->response_slot);
	}

	if (k_uptime_get_32() - throughput_timestamp > THROUGHPUT_PRINT_INTERVAL) {
		int64_t delta = k_uptime_delta(&throughput_timestamp);
		uint64_t measured_kbps = (delta > 0) ?
			(((uint64_t)total_interval_bytes * 8ULL) / (uint64_t)delta) : 0ULL;
		printk("\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
			total_interval_bytes, total_interval_bytes / 1024U, delta, measured_kbps);

		if (g_adv_event_ms_x100_print > 0U && g_num_rsp_slots_print > 0U) {
			uint32_t theo_kbps = (uint32_t)(((uint64_t)g_num_rsp_slots_print *
				g_payload_size_print * 8ULL * 100000ULL) /
				(uint64_t)g_adv_event_ms_x100_print) / 1000U;
			uint32_t efficiency = (theo_kbps > 0U) ?
				(uint32_t)((measured_kbps * 100ULL) / theo_kbps) : 0U;
			printk("[PAwR] theoretical ~%u kbps; efficiency ~%u%% (N=%u, payload=%u, AdvInt=%u.%02u ms)\n",
				(unsigned int)theo_kbps, (unsigned int)efficiency,
				(unsigned int)g_num_rsp_slots_print,
				(unsigned int)g_payload_size_print,
				(unsigned int)(g_adv_event_ms_x100_print / 100U),
				(unsigned int)(g_adv_event_ms_x100_print % 100U));
		}
		total_interval_bytes = 0U;
	}
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.pawr_data_request = request_cb,
	.pawr_response = response_cb,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, ADV_NAME, sizeof(ADV_NAME) - 1),
};

int pawr_broadcaster_init(struct bt_le_ext_adv **out_adv)
{
	int err;

	init_bufs();
	set_pawr_params(&per_adv_params, PAWR_MAX_RESPONSE_SLOTS);

	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &adv_handle);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(adv_handle, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set ext adv data (err %d)\n", err);
		return err;
	}

	err = bt_le_per_adv_set_param(adv_handle, &per_adv_params);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(adv_handle, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return err;
	}

	err = bt_le_per_adv_start(adv_handle);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return err;
	}

	bitmap_clear_all(expected_responses, PAWR_MAX_RESPONSE_SLOTS);
	bitmap_clear_all(response_bitmap, PAWR_MAX_RESPONSE_SLOTS);
	memset(noresp_counters, 0, sizeof(noresp_counters));
	total_interval_bytes = 0U;
	throughput_timestamp = k_uptime_get_32();

	if (out_adv) {
		*out_adv = adv_handle;
	}

	printk("PAwR advertiser started\n");
	return 0;
}

void pawr_broadcaster_set_slot_expected(uint8_t slot, bool expected)
{
	if (slot >= PAWR_MAX_RESPONSE_SLOTS) {
		return;
	}

	if (expected) {
		bitmap_set_bit(expected_responses, slot);
	} else {
		bitmap_clear_bit(expected_responses, slot);
		noresp_counters[slot] = 0U;
	}
}

uint8_t pawr_broadcaster_total_slots(void)
{
	return PAWR_MAX_RESPONSE_SLOTS;
}

