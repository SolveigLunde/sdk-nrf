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
#include <math.h>


/*
* PAwR Throughput Demo
* Advertiser sends subevent data.
* Devices respond in allocated slots.
* Retransmissions are bitmap-controlled.
* GATT is used for sync and slot configuration.
*/

#define NUM_RSP_SLOTS CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define NUM_SUBEVENTS 1
#define PACKET_SIZE   251
#define NAME_LEN      30

#define SYNC_RSP_SIZE 232  // Optimal size for 1.25ms slot alignment
#define THROUGHPUT_PRINT_INTERVAL 1000 
#define UNIT_MS 1.25

static uint32_t total_bytes;
static uint64_t stamp;



void set_pawr_params_new(struct bt_le_per_adv_param *params){


	uint16_t min_units = (uint16_t)(1 + (9 * NUM_RSP_SLOTS + 9) / 10);
	uint8_t interval_units = (min_units < 6 ? 6 : min_units);

	interval_units += 1;

	params->interval_min = interval_units;
	params->interval_max = interval_units;
	params->options = 0;
	params->num_subevents = NUM_SUBEVENTS;
	params->subevent_interval =  interval_units; 
	params->response_slot_delay = 0x01;  
	params->response_slot_spacing = 9;
	params->num_response_slots = NUM_RSP_SLOTS; 

}


static struct bt_le_per_adv_param per_adv_params;

void init_adv_params(void) {
	//set_pawr_params(&per_adv_params, NUM_RSP_SLOTS);
	set_pawr_params_new(&per_adv_params);
}

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

#define BITMAP_BYTES_NEEDED(num_devices) (((num_devices) + 7) / 8)
static uint8_t response_bitmap[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];
static uint8_t expected_responses[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];

static void bitmap_set_bit(uint8_t *bitmap, int bit) {
    bitmap[bit / 8] |= (1 << (bit % 8));
}

static bool bitmap_test_bit(const uint8_t *bitmap, int bit) {
    return (bitmap[bit / 8] & (1 << (bit % 8))) != 0;
}

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

    uint8_t bitmap_bytes = BITMAP_BYTES_NEEDED(NUM_RSP_SLOTS);
    uint8_t retransmit_bitmap[bitmap_bytes];
    
    for (int i = 0; i < bitmap_bytes; i++) {
        retransmit_bitmap[i] = expected_responses[i] & (~response_bitmap[i]);
    }
    
    bool has_retransmits = false;
    for (int i = 0; i < bitmap_bytes; i++) {
        if (retransmit_bitmap[i] != 0) {
            has_retransmits = true;
            break;
        }
    }
    
    if (has_retransmits) {
        printk("Empty response slots detected:\n");
        for (int slot = 0; slot < NUM_RSP_SLOTS; slot++) {
            if (bitmap_test_bit(retransmit_bitmap, slot)) {
                printk("  Slot %d: packet lost\n", slot);
            }
        }
    }

    
    bitmap_clear(response_bitmap, NUM_RSP_SLOTS);

    for (size_t i = 0; i < to_send; i++) {
        buf = &bufs[i];
        
        net_buf_simple_reset(buf);
        
        // Add manufacturer specific data with ACK bitmap
        uint8_t *length_field = net_buf_simple_add(buf, 1);
        net_buf_simple_add_u8(buf, BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(buf, 0x0059); // Nordic Company ID
        
        for (int j = 0; j < bitmap_bytes; j++) {
            net_buf_simple_add_u8(buf, retransmit_bitmap[j]);
        }
        
        *length_field = buf->len - 1;

        subevent_data_params[i].subevent = request->start + i;
        subevent_data_params[i].response_slot_start = 0;
        subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
        subevent_data_params[i].data = buf;
    }

    err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk("Failed to set subevent data (err %d)\n", err);
    } 
}



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

        if (info->response_slot < NUM_RSP_SLOTS) {
            bitmap_set_bit(response_bitmap, info->response_slot);
            
            if (!bitmap_test_bit(expected_responses, info->response_slot)) {
                bitmap_set_bit(expected_responses, info->response_slot);
                printk("Slot %d now actively responding\n", info->response_slot);
            }
        }
        
        if (k_uptime_get_32() - stamp > THROUGHPUT_PRINT_INTERVAL) {
            delta = k_uptime_delta(&stamp);
			

            printk("\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
                   total_bytes, total_bytes / 1024, 
                   delta, ((uint64_t)total_bytes * 8 / delta));
            total_bytes = 0;
        }
    }
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.pawr_data_request = request_cb,
	.pawr_response = response_cb,
};

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

static struct bt_conn *default_conn;

void connected_cb(struct bt_conn *conn, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	__ASSERT(conn == default_conn, "Unexpected connected callback");

	if (err != 0) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	}

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
	printk("Remote info avaliable \n");
	k_sem_give(&sem_connected);
}

void le_param_updated(struct bt_conn *conn, uint16_t interval,
                     uint16_t latency, uint16_t timeout)
{
    printk("Connection parameters updated: interval %.2f ms, latency %d, timeout %d ms\n",
           (double)(interval * 1.25f), latency, timeout * 10);
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

	if (strcmp(name, "PAwR_Adv")) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
	}
	
}


static struct bt_uuid_128 pawr_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
static uint16_t pawr_attr_handle;

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params)
{
	struct bt_gatt_chrc *chrc;
	char str[BT_UUID_STR_LEN];

	printk("[GATT] Discovery callback with attr: %p\n", attr);

	if (!attr) {
		printk("[GATT] No more attributes, discovery complete\n");
		return BT_GATT_ITER_STOP;
	}

	if (!attr->user_data) {
		printk("[GATT] Attribute has no user data\n");
		return BT_GATT_ITER_CONTINUE;
	}

	chrc = (struct bt_gatt_chrc *)attr->user_data;

	bt_uuid_to_str(chrc->uuid, str, sizeof(str));
	printk("[GATT] Found characteristic with UUID: %s\n", str);
	printk("[GATT] Properties: 0x%02X, Value Handle: %d\n", 
		chrc->properties, chrc->value_handle);

	if (!bt_uuid_cmp(chrc->uuid, &pawr_char_uuid.uuid)) {
		pawr_attr_handle = chrc->value_handle;
		printk("[GATT] Found PAwR characteristic! Handle: %d\n", pawr_attr_handle);
		k_sem_give(&sem_discovered);
		return BT_GATT_ITER_STOP;
	}

	printk("[GATT] Not our characteristic, continuing...\n");
	return BT_GATT_ITER_CONTINUE;
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
		net_buf_simple_init_with_data(&bufs[i], &backing_store[i],ARRAY_SIZE(backing_store[i]));
		net_buf_simple_reset(&bufs[i]);
		/* Add manufacturer specific data */
		net_buf_simple_add_u8(&bufs[i], 3); /* Length of manufacturer data */
		net_buf_simple_add_u8(&bufs[i], BT_DATA_MANUFACTURER_DATA);
		net_buf_simple_add_le16(&bufs[i], 0x0059); /* Nordic Company ID */
		
		printk("Buffer %d initialized with len %d\n", i, bufs[i].len);
	}
}


struct pawr_timing {
	uint8_t subevent;
	uint8_t response_slot;
	uint16_t total_devices; 
} __packed;

static uint8_t num_synced;

static const struct bt_data ad_conn[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "PAwR_Adv", 8),
    /* Optionally: add UUIDs for your PAwR timing GATT service */
};

static const uint8_t pawr_payload[] = {0x01, 0x02, 0x03, 0x04};

int main(void)
{
	int err;
	struct bt_le_ext_adv *pawr_adv;
	struct bt_le_ext_adv *adv_conn;
	struct bt_gatt_discover_params discover_params;
	struct bt_gatt_write_params write_params;
	struct pawr_timing sync_config;

	init_bufs();

	printk("Starting PAwR Advertiser\n");
	printk("===========================================\n");


	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	init_adv_params();
	/* Connectable adv set for GATT/PAST handshake */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv_conn);
	if (err) {
		printk("Failed to create connectable advertising set (err %d)\n", err);
		return 0;
	}

	/* set advertising data for the connectable set (e.g. local name or service UUIDs) */
	err = bt_le_ext_adv_set_data(adv_conn, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
	if (err) {
		printk("Failed to set connectable adv data (err %d)\n", err);
	}


	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}
	 
	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return 0;
	}

	printk("Start Extended Advertising\n");
	err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	err = bt_le_per_adv_set_data(pawr_adv, (struct bt_data[]){BT_DATA(BT_DATA_MANUFACTURER_DATA, pawr_payload, sizeof(pawr_payload))}, 1);
	if (err){
		printk("Failed to set per adv data (err %d) \n", err);
		return 0;
	}

	printk("Start connectable advertising \n");
	err = bt_le_per_adv_start(adv_conn);
	if (err) {
		printk("Failed to enable connectable periodic advertising (err %d)\n", err);
		return 0;
	}

	/* Enable Periodic Advertising */
	printk("Start Periodic Advertising\n");
	err = bt_le_per_adv_start(pawr_adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return 0;
	}

	while (num_synced < CONFIG_BT_MAX_THROUGHPUT_DEVICES) {
		/* Enable continuous scanning */
		k_sleep(K_MSEC(10));
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

		printk("[PAST] Initiating PAST transfer...\n");
		err = bt_le_per_adv_set_info_transfer(pawr_adv, default_conn, 0);
		if (err) {
			printk("[PAST] Failed to send PAST (err %d)\n", err);
			if (err == -EINVAL) {
				printk("[PAST] Error: Invalid parameters\n");
			} else if (err == -ENOTCONN) {
				printk("[PAST] Error: Connection lost\n");
			} else if (err == -EBUSY) {
				printk("[PAST] Error: Controller busy\n");
			}

			goto disconnect;
		}
		printk("[PAST] Transfer initiated successfully\n");

		printk("[GATT] Starting characteristic discovery...\n");
		discover_params.uuid = &pawr_char_uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		printk("[GATT] Looking for UUID: ");
		char uuid_str[BT_UUID_STR_LEN];
		bt_uuid_to_str(&pawr_char_uuid.uuid, uuid_str, sizeof(uuid_str));
		printk("%s\n", uuid_str);

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("[GATT] Discovery failed to start (err %d)\n", err);
			if (err == -EINVAL) {
				printk("[GATT] Invalid parameters\n");
			} else if (err == -ENOTCONN) {
				printk("[GATT] Not connected\n");
			} else if (err == -EALREADY) {
				printk("[GATT] Discovery already in progress\n");
			}
			goto disconnect;
		}
		printk("[GATT] Discovery procedure started successfully\n");

		printk("[GATT] Waiting for discovery to complete (timeout: 10s)...\n");
		err = k_sem_take(&sem_discovered, K_SECONDS(10));
		if (err) {
			printk("[GATT] ERROR: Discovery timed out after 10 seconds!\n");
			printk("[GATT] Last known state: Discovery started but no callback received\n");
			goto disconnect;
		}
		printk("[GATT] Discovery completed successfully\n");

		sync_config.subevent = 0; 
		sync_config.response_slot = num_synced;  
		sync_config.total_devices = CONFIG_BT_MAX_THROUGHPUT_DEVICES;
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
