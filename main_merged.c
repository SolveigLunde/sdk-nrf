/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdint.h>

/* Add throughput measurement variables */
static uint32_t total_bytes;
static uint64_t stamp;
#define THROUGHPUT_PRINT_DURATION 1000 /* Print every second */
#define DEBUG_VERBOSE 0

/* Configuration from Kconfig */
#define NUM_RSP_SLOTS CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define NUM_SUBEVENTS 1
#define PACKET_SIZE   251
#define NAME_LEN      30

#define MAX_BUFFER_SIZE 73  /* Maximum safe buffer size before HCI error */

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovered, 0, 1);
static K_SEM_DEFINE(sem_written, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

struct k_poll_event events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
					&sem_connected, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
					&sem_disconnected, 0),
};

static struct bt_uuid_128 pawr_char_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
static uint16_t pawr_attr_handle;

// Add bitmap tracking variables for retransmission
static uint32_t response_bitmap = 0;  // Bitmap to track which slots have responded
static uint32_t expected_responses = 0;  // Bitmap of expected responses (all slots with synced devices)

void calculate_pawr_params(struct bt_le_per_adv_param *params, uint8_t num_response_slots)
{
	// Start with proven working values from working_retransmission
	const uint8_t PROVEN_RESPONSE_SLOT_SPACING = 0x04;  /* 8 * 0.125ms = 1ms */
	const uint8_t PROVEN_RESPONSE_SLOT_DELAY = 0x18;    /* 16 * 1.25ms = 20ms */
	const uint8_t PROVEN_SUBEVENT_INTERVAL = 0x30;      /* 32 * 1.25ms = 40ms */
	const uint8_t PROVEN_INTERVAL = 0x40;               /* 48 * 1.25ms = 60ms */
	
	// Dynamic calculation constants
	const uint8_t MIN_RESPONSE_SLOT_SPACING_UNITS = 4;   
	const uint8_t MIN_RESPONSE_SLOT_DELAY_UNITS = 8;     
	const uint8_t MARGIN_MS = 10;

	uint8_t slot_spacing = MIN_RESPONSE_SLOT_SPACING_UNITS;
	uint8_t delay = MIN_RESPONSE_SLOT_DELAY_UNITS;

	float total_time_ms = delay * 1.25 + num_response_slots * slot_spacing * 0.125 + MARGIN_MS;
	uint8_t subevent_interval = (uint8_t)((total_time_ms * 1000 + 1249) / 1250);

	// Clamp to BLE spec limits and respect configured interval
	subevent_interval = CLAMP(subevent_interval, 6, 255);
	uint16_t configured_interval = CONFIG_BT_MAX_THROUGHPUT_PAWR_INTERVAL_MS / 1.25;
	if (configured_interval > subevent_interval) {
		subevent_interval = MIN(configured_interval, 255);
	}

	// Use dynamic calculation, but ensure values are not more aggressive than proven working values
	params->interval_min = MAX(subevent_interval, PROVEN_INTERVAL);
	params->interval_max = params->interval_min;
	params->options = BT_LE_ADV_OPT_USE_TX_POWER;
	params->num_subevents = 1;
	params->subevent_interval = MAX(subevent_interval, PROVEN_SUBEVENT_INTERVAL);
	params->response_slot_delay = MAX(delay, PROVEN_RESPONSE_SLOT_DELAY);
	params->response_slot_spacing = MAX(slot_spacing, PROVEN_RESPONSE_SLOT_SPACING);
	params->num_response_slots = num_response_slots;

	printk("PAwR config: %u slots, spacing=%.2fms, delay=%.2fms, interval=%.2fms\n",
			num_response_slots,
			params->response_slot_spacing * 0.125,
			params->response_slot_delay * 1.25,
			params->subevent_interval * 1.25);
}

static struct bt_le_per_adv_param per_adv_params;

void init_adv_params(void) {
    calculate_pawr_params(&per_adv_params, NUM_RSP_SLOTS);
}

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

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
    uint32_t retransmit_bitmap = expected_responses & (~response_bitmap);
    
    // Debug: print empty slots
    if (retransmit_bitmap) {
        printk("Empty response slots detected: 0x%08X\n", retransmit_bitmap);
        for (int slot = 0; slot < NUM_RSP_SLOTS; slot++) {
            if (retransmit_bitmap & (1 << slot)) {
                printk("  Slot %d: packet lost\n", slot);
            }
        }
    }

    // Clear response bitmap for next cycle
    response_bitmap = 0;

    // Process each subevent
    for (size_t i = 0; i < to_send; i++) {
        buf = &bufs[i];
        
        // Reset buffer for each request
        net_buf_simple_reset(buf);
        
        // Add manufacturer specific data with ACK bitmap
        uint8_t *length_field = net_buf_simple_add(buf, 1);
        net_buf_simple_add_u8(buf, BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(buf, 0x0059); // Nordic Company ID
        
        // Add retransmission bitmap (3 bytes for 20 slots)
        net_buf_simple_add_u8(buf, (retransmit_bitmap >> 0) & 0xFF);
        net_buf_simple_add_u8(buf, (retransmit_bitmap >> 8) & 0xFF);
        net_buf_simple_add_u8(buf, (retransmit_bitmap >> 16) & 0xFF);
        
        // Update length field (excluding the length byte itself)
        *length_field = buf->len - 1;

        subevent_data_params[i].subevent = request->start + i;
        subevent_data_params[i].response_slot_start = 0;
        subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
        subevent_data_params[i].data = buf;

        if (DEBUG_VERBOSE) {
            printk("SE%d: len=%d, retransmit_bitmap=0x%08X\n", i, buf->len, retransmit_bitmap);
        }
    }

    err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk("Failed to set subevent data (err %d)\n", err);
    } else if (DEBUG_VERBOSE) {
        printk("Data set OK, retransmit_bitmap=0x%08X\n", retransmit_bitmap);
    }
}

static bool print_ad_field(struct bt_data *data, void *user_data)
{
	ARG_UNUSED(user_data);

	printk("    0x%02X: ", data->type);
	for (size_t i = 0; i < data->data_len; i++) {
		printk("%02X", data->data[i]);
	}

	printk("\n");

	return true;
}

static struct bt_conn *default_conn;

static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
                     struct net_buf_simple *buf)
{
    int64_t delta;

    if (buf) {
        /* Initialize timestamp on first response */
        if (total_bytes == 0) {
            stamp = k_uptime_get_32();
        }

        total_bytes += buf->len;
        
        // Mark this response slot as received
        if (info->response_slot < NUM_RSP_SLOTS) {
            response_bitmap |= (1 << info->response_slot);
            
            // Only expect responses after first successful response
            if (!(expected_responses & (1 << info->response_slot))) {
                expected_responses |= (1 << info->response_slot);
                printk("Slot %d now actively responding\n", info->response_slot);
            }
        }
        
        if (k_uptime_get_32() - stamp > THROUGHPUT_PRINT_DURATION) {
            delta = k_uptime_delta(&stamp);

            printk("\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
                   total_bytes, total_bytes / 1024, 
                   delta, ((uint64_t)total_bytes * 8 / delta));
            FILE *log = fopen("throughput.log", "a");
            if (log) {
                fprintf(log, "\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",total_bytes, 
                    total_bytes / 1024, delta, ((uint64_t)total_bytes * 8 / delta));
                fclose(log);
            }
            total_bytes = 0;
        }

        printk("Response: subevent %d, slot %d\n", info->subevent, info->response_slot);
        bt_data_parse(buf, print_ad_field, NULL);
    }
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.pawr_data_request = request_cb,
	.pawr_response = response_cb,
};

void connected_cb(struct bt_conn *conn, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	__ASSERT(conn == default_conn, "Unexpected connected callback");

	if (err) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	}

	/* Request 2M PHY */
	struct bt_conn_le_phy_param phy_param = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
	};
	
	err = bt_conn_le_phy_update(conn, &phy_param);
	if (err) {
		printk("PHY update request failed (err %d)\n", err);
	} else {
		printk("PHY update request sent\n");
	}
}

void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02X %s\n", reason, bt_hci_err_to_str(reason));

	k_sem_give(&sem_disconnected);
}

void remote_info_available_cb(struct bt_conn *conn, struct bt_conn_remote_info *remote_info)
{
	/* Need to wait for remote info before initiating PAST */
	k_sem_give(&sem_connected);
}

static const char *phy_to_str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_1M:
		return "LE 1M";
	case BT_GAP_LE_PHY_2M:
		return "LE 2M";
	case BT_GAP_LE_PHY_CODED:
		return "LE Coded";
	default:
		return "Unknown";
	}
}

void le_param_updated(struct bt_conn *conn, uint16_t interval,
                     uint16_t latency, uint16_t timeout)
{
    printk("Connection parameters updated: interval %.2f ms, latency %d, timeout %d ms\n",
           interval * 1.25f, latency, timeout * 10);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.remote_info_available = remote_info_available_cb,
	.le_param_updated = le_param_updated,
};

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	(void)memset(name, 0, sizeof(name));
	bt_data_parse(ad, data_cb, name);

	if (strcmp(name, "PAwR sync sample")) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&default_conn);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
	}
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	struct bt_gatt_chrc *chrc;
	char str[BT_UUID_STR_LEN];

	printk("Discovery: attr %p\n", attr);

	if (!attr) {
		return BT_GATT_ITER_STOP;
	}

	chrc = (struct bt_gatt_chrc *)attr->user_data;

	bt_uuid_to_str(chrc->uuid, str, sizeof(str));
	printk("UUID %s\n", str);

	if (!bt_uuid_cmp(chrc->uuid, &pawr_char_uuid.uuid)) {
		pawr_attr_handle = chrc->value_handle;

		printk("Characteristic handle: %d\n", pawr_attr_handle);

		k_sem_give(&sem_discovered);
	}

	return BT_GATT_ITER_STOP;
}

static void write_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	if (err) {
		printk("Write failed (err %d)\n", err);

		return;
	}

	k_sem_give(&sem_written);
}

void init_bufs(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(backing_store); i++) {
        /* Initialize the buffer first */
        net_buf_simple_init_with_data(&bufs[i], &backing_store[i],
                                  ARRAY_SIZE(backing_store[i]));
        
        net_buf_simple_reset(&bufs[i]);
        
        /* Add manufacturer specific data */
        net_buf_simple_add_u8(&bufs[i], 3); /* Length of manufacturer data */
        net_buf_simple_add_u8(&bufs[i], BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(&bufs[i], 0x0059); /* Nordic Company ID */
        
        printk("Buffer %d initialized with len %d\n", i, bufs[i].len);
    }
}

#define MAX_SYNCS (NUM_SUBEVENTS * NUM_RSP_SLOTS)
struct pawr_timing {
	uint8_t subevent;
	uint8_t response_slot;
} __packed;

static uint8_t num_synced;

int main(void)
{
	int err;
	struct bt_le_ext_adv *pawr_adv;
	struct bt_gatt_discover_params discover_params;
	struct bt_gatt_write_params write_params;
	struct pawr_timing sync_config;

	init_bufs();

	printk("Starting Periodic Advertising Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	/* Initialize advertising parameters with dynamic calculation */
	init_adv_params();

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return 0;
	}

	/* Enable Periodic Advertising */
	printk("Start Periodic Advertising\n");
	err = bt_le_per_adv_start(pawr_adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return 0;
	}

	printk("Start Extended Advertising\n");
	err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	while (num_synced < MAX_SYNCS) {
		/* Enable continuous scanning */
		err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
		if (err) {
			printk("Scanning failed to start (err %d)\n", err);
			return 0;
		}

		printk("Scanning successfully started\n");

		/* Wait for either remote info available or involuntary disconnect */
		k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		err = k_sem_take(&sem_connected, K_NO_WAIT);
		if (err) {
			printk("Disconnected before remote info available\n");

			goto disconnected;
		}

		err = bt_le_per_adv_set_info_transfer(pawr_adv, default_conn, 0);
		if (err) {
			printk("Failed to send PAST (err %d)\n", err);

			goto disconnect;
		}

		printk("PAST sent\n");

		discover_params.uuid = &pawr_char_uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discovery failed (err %d)\n", err);

			goto disconnect;
		}

		printk("Discovery started\n");

		err = k_sem_take(&sem_discovered, K_SECONDS(10));
		if (err) {
			printk("Timed out during GATT discovery\n");

			goto disconnect;
		}

		sync_config.subevent = num_synced % NUM_SUBEVENTS;
		sync_config.response_slot = num_synced / NUM_SUBEVENTS;
		num_synced++;

		write_params.func = write_func;
		write_params.handle = pawr_attr_handle;
		write_params.offset = 0;
		write_params.data = &sync_config;
		write_params.length = sizeof(sync_config);

		err = bt_gatt_write(default_conn, &write_params);
		if (err) {
			printk("Write failed (err %d)\n", err);
			num_synced--;

			goto disconnect;
		}

		printk("Write started\n");

		err = k_sem_take(&sem_written, K_SECONDS(10));
		if (err) {
			printk("Timed out during GATT write\n");
			num_synced--;

			goto disconnect;
		}

		printk("PAwR config written to sync %d, disconnecting\n", num_synced - 1);

disconnect:
		/* Adding delay (2ms * interval value, using 2ms intead of the 1.25ms
		 * used by controller) to ensure sync is established before
		 * disconnection.
		 */
		k_sleep(K_MSEC(per_adv_params.interval_max * 2));

		err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (err != 0 && err != -ENOTCONN) {
			return 0;
		}

disconnected:
		k_sem_take(&sem_disconnected, K_FOREVER);

		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	printk("Maximum numnber of syncs onboarded\n");

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
} 