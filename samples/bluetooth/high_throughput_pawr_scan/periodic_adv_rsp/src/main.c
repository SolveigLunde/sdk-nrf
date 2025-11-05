/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/*
* PAwR Throughput Demo
* Advertiser sends subevent data with control information.
* Devices claim response slots using PAwR responses (no GATT/PAST used).
* Retransmissions are bitmap-controlled.
*/

#define NUM_RSP_SLOTS CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define NUM_SUBEVENTS 1
#define PAYLOAD_SIZE   251
#define ADV_NAME "PAwR adv sample"

#define MAX_INDIVIDUAL_RESPONSE_SIZE 247 // BLE spec limit for individual responses
#define THROUGHPUT_PRINT_INTERVAL 1000 

static uint32_t total_bytes;
static int64_t stamp;
 

void set_pawr_params(struct bt_le_per_adv_param *params, uint8_t num_response_slots)
{
    /* BLE spec / Zephyr API constraints (see bt_le_per_adv_param docs) */
    const uint16_t MIN_PAWR_INTERVAL_MS = 1;             
    const float ADVERTISER_GUARD_MS = 1.0f;     
    const uint8_t PHY_RATE_MBPS = 2;         

    /* Max per-slot payload for PAwR responses */
    const uint16_t payload_size = MAX_INDIVIDUAL_RESPONSE_SIZE; 

    /* Approximate on-air time + radio turnarounds at 2M PHY */
    const float tx_time_ms = (float)(payload_size * 8) / (PHY_RATE_MBPS * 1000.0f);
    const float slot_time_ms = tx_time_ms + 0.25f; 

    /* Convert per-slot time to spacing in 0.125 ms units, clamp per API (0.25..31.875 ms) */
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
                                              (uint32_t)((ADVERTISER_GUARD_MS) * 100.0f);

    const uint16_t min_interval_units = (uint16_t)((MIN_PAWR_INTERVAL_MS + 1) / 1.25f); /* min interval (1.25 ms units) */
    const uint16_t required_interval_units = (uint16_t)((total_event_time_ms_x100 + 125U - 1U) / 125U);
    uint16_t advertising_event_interval_units = (required_interval_units > min_interval_units)
                                                          ? required_interval_units
                                                          : min_interval_units;

    if (advertising_event_interval_units < subevent_interval_units) {
        advertising_event_interval_units = subevent_interval_units;
    }

    /* One subevent, N slots, maximum payload per slot */
    params->interval_min = advertising_event_interval_units;
    params->interval_max = advertising_event_interval_units;
    params->options = 0;
    params->num_subevents = 1;
    params->subevent_interval = (uint8_t)subevent_interval_units;
    params->response_slot_delay = delay_units;
    params->response_slot_spacing = (uint8_t)slot_spacing_units;
    params->num_response_slots = num_response_slots;

    /* Diagnostics */
    const uint32_t adv_event_ms_x100 = (uint32_t)advertising_event_interval_units * 125U;
    const uint32_t subevent_ms_x100 = (uint32_t)subevent_interval_units * 125U;
    const uint32_t slot_ms_x100 = (uint32_t)slot_spacing_units * 125U / 10U;
    const uint32_t est_throughput_kbps =
        (adv_event_ms_x100 > 0U)
            ? (uint32_t)(((uint64_t)num_response_slots * payload_size * 8ULL * 100000ULL) /
                         (uint64_t)adv_event_ms_x100) /
                  1000U
            : 0U;

    printk("PAwR config (1 subevent):\n");
    printk("  Devices: %u, Resp size: %u B, Slot: %u.%02u ms\n",
           num_response_slots, payload_size, slot_ms_x100 / 100U, slot_ms_x100 % 100U);
    printk("  Delay: %u (%u.%02u ms), SubEvt: %u (%u.%02u ms), AdvInt: %u (%u.%02u ms)\n",
           delay_units, (uint32_t)delay_units * 125U / 100U, (uint32_t)delay_units * 125U % 100U,
           subevent_interval_units, subevent_ms_x100 / 100U, subevent_ms_x100 % 100U,
           advertising_event_interval_units, adv_event_ms_x100 / 100U, adv_event_ms_x100 % 100U);
	printk("  Est. uplink throughput (theoretical) ~%u kbps\n", (unsigned int)est_throughput_kbps);
}


static struct bt_le_per_adv_param per_adv_params;

void init_adv_params(void) {
	set_pawr_params(&per_adv_params,NUM_RSP_SLOTS);
}

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PAYLOAD_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

#define BITMAP_BYTES_NEEDED(num_devices) (((num_devices) + 7) / 8)
static uint8_t response_bitmap[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];
static uint8_t expected_responses[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];

/*
 * Slot assignment state and ACK queue for claim/ack protocol
 */
struct slot_assignment_state {
    bool assigned;
    uint32_t token; /* Token that owns this slot, 0 if unassigned */
};

static struct slot_assignment_state slot_state[NUM_RSP_SLOTS];

struct ack_entry {
    uint32_t token;
    uint8_t slot;
};

#define MAX_ACKS_PER_EVENT 32
static struct ack_entry ack_queue[MAX_ACKS_PER_EVENT];
static uint8_t ack_queue_count;

/* Build a bitmap of unassigned (open) slots; Bit i set => slot i is open*/
static void build_open_slots_bitmap(uint8_t *out, uint8_t bytes)
{
    memset(out, 0, bytes);
    for (int i = 0; i < NUM_RSP_SLOTS; i++) {
        if (!slot_state[i].assigned) {
            uint8_t idx = (uint8_t)(i / 8);
            uint8_t bit = (uint8_t)(i % 8);
            if (idx < bytes) {
                out[idx] |= (uint8_t)(1U << bit);
            }
        }
    }
}

/* Linear search for slot by token owner*/
static int find_slot_by_token(uint32_t token)
{
    if (token == 0) {
        return -1;
    }
    for (int i = 0; i < NUM_RSP_SLOTS; i++) {
        if (slot_state[i].assigned && slot_state[i].token == token) {
            return i;
        }
    }
    return -1;
}

/* Append an ACK (tocken,slot) if queue has room */
static void queue_ack(uint32_t token, uint8_t slot)
{
    if (ack_queue_count < MAX_ACKS_PER_EVENT) {
        ack_queue[ack_queue_count].token = token;
        ack_queue[ack_queue_count].slot = slot;
        ack_queue_count++;
    }
}

/* Set bit in a byte-addressed bitmap*/
static void bitmap_set_bit(uint8_t *bitmap, int bit) {
    /* Test bit in a byte-addressed bitmap*/
    bitmap[bit / 8] |= (1 << (bit % 8));
}

/* Clear bitmap to zero for the first num_bits bits */
static bool bitmap_test_bit(const uint8_t *bitmap, int bit) {
    return (bitmap[bit / 8] & (1 << (bit % 8))) != 0;
}

/* Build PAwR control payload and program controller*/
static void bitmap_clear(uint8_t *bitmap, int num_bits) {
    memset(bitmap, 0, BITMAP_BYTES_NEEDED(num_bits));
}

static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
    int err;
    uint8_t to_send;
    struct net_buf_simple *buf;

    if (!request) {
        printk("Error: NULL request received\n");
        return;
    }

    to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

    // Calculate retransmission bitmap (bits set for slots that didn't respond)
    uint8_t bitmap_bytes = BITMAP_BYTES_NEEDED(NUM_RSP_SLOTS);
    uint8_t retransmit_bitmap[bitmap_bytes];
    
    for (int i = 0; i < bitmap_bytes; i++) {
        retransmit_bitmap[i] = expected_responses[i] & (~response_bitmap[i]);
    }
    
    /* Idle-slot reclamation: if slot is expected but no response observed for N events, free it */
    static uint8_t noresp_counters[NUM_RSP_SLOTS];
    for (int slot = 0; slot < NUM_RSP_SLOTS; slot++) {
        bool was_expected = bitmap_test_bit(expected_responses, slot);
        bool got_response = bitmap_test_bit(response_bitmap, slot);
        if (was_expected && !got_response) {
            if (noresp_counters[slot] < 255) {
                noresp_counters[slot]++;
            }
            if (noresp_counters[slot] >= 3) {
                /* Consider this slot idle; free it for new claimers */
                slot_state[slot].assigned = false;
                slot_state[slot].token = 0;
                noresp_counters[slot] = 0;
                printk("[CLAIM] Reclaimed idle slot %d\n", slot);
            }
        } else if (got_response) {
            noresp_counters[slot] = 0;
        }
    }

    
    bitmap_clear(response_bitmap, NUM_RSP_SLOTS);

    // Process each subevent
    for (size_t i = 0; i < to_send; i++) {
        buf = &bufs[i];
        net_buf_simple_reset(buf);
        
        uint8_t *length_field = net_buf_simple_add(buf, 1);
        net_buf_simple_add_u8(buf, BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(buf, 0x0059); // Nordic Company ID
        net_buf_simple_add_u8(buf, 0x01); // protocol version
        net_buf_simple_add_u8(buf, 0x00); // protocol flags (none)

        /* Open slots bitmap */
        uint8_t open_len_full = BITMAP_BYTES_NEEDED(NUM_RSP_SLOTS);
        uint8_t open_len = open_len_full > 32 ? 32 : open_len_full;
        uint8_t tmp_open[32];
        build_open_slots_bitmap(tmp_open, open_len);
        net_buf_simple_add_u8(buf, open_len);
        for (uint8_t j = 0; j < open_len; j++) {
            net_buf_simple_add_u8(buf, tmp_open[j]);
        }

        /* ACK list: pack as many as tailroom allows, but cap to MAX_ACKS_PER_EVENT */
        uint8_t *ack_count_ptr = net_buf_simple_add(buf, 1);
        *ack_count_ptr = 0;
        size_t reserved_for_rt = 1 /* rt_len */ + bitmap_bytes /* rt bytes */;
        size_t tail = net_buf_simple_tailroom(buf);
        size_t ack_space = (tail > reserved_for_rt) ? (tail - reserved_for_rt) : 0;
        uint8_t ack_capacity = (uint8_t)(ack_space / 5); /* token(4)+slot(1) */
        if (ack_capacity > MAX_ACKS_PER_EVENT) {
            ack_capacity = MAX_ACKS_PER_EVENT;
        }
        uint8_t acks_to_send = MIN(ack_queue_count, ack_capacity);
        for (uint8_t a = 0; a < acks_to_send; a++) {
            net_buf_simple_add_le32(buf, ack_queue[a].token);
            net_buf_simple_add_u8(buf, ack_queue[a].slot);
        }
        *ack_count_ptr = acks_to_send;

        /* Retransmit bitmap: clamp to 4 bytes to keep control small during join */
        uint8_t rt_bytes_to_send = bitmap_bytes > 4 ? 4 : bitmap_bytes;
        net_buf_simple_add_u8(buf, rt_bytes_to_send);
        for (int j = 0; j < rt_bytes_to_send; j++) {
            net_buf_simple_add_u8(buf, retransmit_bitmap[j]);
        }

        *length_field = buf->len - 1;

        subevent_data_params[i].subevent = request->start + i;
        subevent_data_params[i].response_slot_start = 0;
        subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
        subevent_data_params[i].data = buf;
    }

    /* We just consumed pending ACKs; clear the queue for next event */
    ack_queue_count = 0;

    err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk(" [ADV]Failed to set subevent data (err %d)\n", err);
    } 
}



static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
                     struct net_buf_simple *buf)
{
    int64_t delta;

    if (buf) {
        /* Initialize timestamp on first response */
        if (total_bytes == 0) {
            stamp = k_uptime_get();
        }

        /* Handle claim messages or data responses */
        if (buf->len >= 5 && buf->data[0] == 0xC1) {
            /* CLAIM message: [0xC1][token(4B LE)] */
            uint32_t token = sys_get_le32(&buf->data[1]);

            if (info->response_slot < NUM_RSP_SLOTS) {
                int already = find_slot_by_token(token);
                if (already >= 0) {
                    /* This token already owns a slot: only ACK its true slot */
                    if (already == info->response_slot) {
                        queue_ack(token, (uint8_t)already);
                    }
                } else if (!slot_state[info->response_slot].assigned) {
                    /* Assign the slot to this token and queue ACK */
                    slot_state[info->response_slot].assigned = true;
                    slot_state[info->response_slot].token = token;
                    bitmap_set_bit(expected_responses, info->response_slot);
                    queue_ack(token, info->response_slot);
                    printk("[CLAIM] accepted: slot %d token 0x%08x\n", info->response_slot, token);
                } else {
                    /* Slot already assigned to a different token: ignore */
                }
            }
        } else {
            /* Count payload bytes as throughput */
            total_bytes += buf->len;
        }

        if (info->response_slot < NUM_RSP_SLOTS) {
            bitmap_set_bit(response_bitmap, info->response_slot);
        }
        

        if (k_uptime_get_32() - stamp > THROUGHPUT_PRINT_INTERVAL) {
            delta = k_uptime_delta(&stamp);
			/* Measured kbps over the window */
			uint64_t measured_kbps = (delta > 0) ? (((uint64_t)total_bytes * 8ULL) / (uint64_t)delta) : 0ULL;
			printk("\n[ADV] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
			   total_bytes, total_bytes / 1024, delta, measured_kbps);
            
            uint16_t interval_units = per_adv_params.interval_min; 
            uint32_t adv_ms_x100 = (uint32_t)interval_units * 125U;
            uint16_t num_rsp_slots = per_adv_params.num_response_slots;
            uint16_t payload_bytes = MAX_INDIVIDUAL_RESPONSE_SIZE;
            
            if (adv_ms_x100 > 0U && num_rsp_slots > 0U) {
                uint32_t theo_kbps =
                    (uint32_t)(((uint64_t)num_rsp_slots * payload_bytes * 8ULL * 100000ULL) /
                                (uint64_t)adv_ms_x100) / 1000U;
            
                printk("[PAwR] theoretical ~%u kbps; efficiency ~%u%% (N=%u, payload=%u, AdvInt=%u.%02u ms)\n",
                        (unsigned int)theo_kbps,
                        (unsigned int)((theo_kbps > 0U) ? (uint32_t)((measured_kbps * 100ULL) / theo_kbps) : 0U),
                        (unsigned int)num_rsp_slots,
                        (unsigned int)payload_bytes,
                        (unsigned int)(adv_ms_x100 / 100U),
                        (unsigned int)(adv_ms_x100 % 100U));
            }
            total_bytes = 0;
        }
    }
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.pawr_data_request = request_cb,
	.pawr_response = response_cb,
};


void init_bufs(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(backing_store); i++) {
		/* Initialize buffer with backing storage */
		net_buf_simple_init_with_data(&bufs[i], &backing_store[i],
								ARRAY_SIZE(backing_store[i]));
		
		net_buf_simple_reset(&bufs[i]);
		
		/* Add manufacturer specific data */
		net_buf_simple_add_u8(&bufs[i], 3); /* Length of manufacturer data */
		net_buf_simple_add_u8(&bufs[i], BT_DATA_MANUFACTURER_DATA);
		net_buf_simple_add_le16(&bufs[i], 0x0059); /* Nordic Company ID */
		
		printk("[ADV] Buffer %zu initialized with len %u\n", i, bufs[i].len);
	}
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, ADV_NAME, sizeof(ADV_NAME) - 1),
};

int main(void)
{
	int err;
	struct bt_le_ext_adv *pawr_adv;

    // Initialize buffers
	init_bufs();

    // Initialize Bluetooth
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	// Initialize parameters
	init_adv_params();

    printk("Starting PAwR Advertiser\n");
	printk("===========================================\n");

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

    /* Set extended advertising data */
	err = bt_le_ext_adv_set_data(pawr_adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Failed to set ext adv data (err %d)\n", err);
        return 0;
    }

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return 0;
	}

    /* Enable Extended Advertising */
    printk("Start Extended Advertising\n");
	err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	/* Enable Periodic Advertising */
	printk("Start Periodic Advertising\n");
	err = bt_le_per_adv_start(pawr_adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return 0;
	}

	return 0;
}
