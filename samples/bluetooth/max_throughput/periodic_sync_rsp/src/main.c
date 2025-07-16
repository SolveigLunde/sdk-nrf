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

/* Add throughput measurement variables */
static uint32_t total_bytes;
static uint64_t stamp;
#define THROUGHPUT_PRINT_DURATION 1000 /* Print every second */

#define PACKET_SIZE 251

static struct bt_uuid_128 pawr_char_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_gatt_service_static pawr_svc = BT_GATT_SERVICE_STATIC_DEFINE(
	BT_GATT_PRIMARY_SERVICE(&pawr_char_uuid),
	BT_GATT_CHARACTERISTIC(&pawr_char_uuid.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, write_pawr_char, NULL));

static struct bt_le_per_adv_sync *sync;
static struct bt_le_per_adv_sync_subevent_params subevent_params;
static uint8_t subevent_data[PACKET_SIZE];
static uint8_t subevent;
static uint8_t response_slot;

static void subevent_data_cb(struct bt_le_per_adv_sync *sync,
			     const struct bt_le_per_adv_sync_subevent_info *info,
			     struct net_buf_simple *buf)
{
	printk("Subevent %d\n", info->subevent);
	if (buf) {
		printk("  tx_power: %d\n", info->tx_power);
		printk("  rssi: %d\n", info->rssi);
		printk("  cte_type: %d\n", info->cte_type);
		printk("  data_status: %d\n", info->data_status);
		printk("  data_len: %d\n", buf->len);
		printk("  data: ");
		for (int i = 0; i < buf->len; i++) {
			printk("%02x ", buf->data[i]);
		}
		printk("\n");
	}
}

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	printk("PER_ADV_SYNC[%d]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u us), PHY %s\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(&info->addr),
	       info->interval, BT_CONN_INTERVAL_TO_US(info->interval),
	       phy_str(info->phy));
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	printk("PER_ADV_SYNC[%d]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(&info->addr));
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	int64_t delta;
	
	if (buf) {
		/* Initialize timestamp on first response */
		if (total_bytes == 0) {
			stamp = k_uptime_get_32();
		}

		total_bytes += buf->len;
		
		/* Print throughput every second */
		if (k_uptime_get_32() - stamp > THROUGHPUT_PRINT_DURATION) {
			delta = k_uptime_delta(&stamp);
			
			printk("\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
			       total_bytes, total_bytes / 1024, 
			       delta, ((uint64_t)total_bytes * 8 / delta));
			
			FILE *log = fopen("throughput.log", "a");
			if (log) {
				fprintf(log, "\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
				       total_bytes, total_bytes / 1024, 
				       delta, ((uint64_t)total_bytes * 8 / delta));
				fclose(log);
			}
			
			/* Reset counters for next interval */
			total_bytes = 0;
		}

		printk("Received %d bytes\n", buf->len);
		
		// Parse manufacturer data to check for retransmission bitmap
		if (buf->len >= 7) { // Minimum length for manufacturer data with bitmap
			uint8_t *data = buf->data;
			uint8_t length = data[0];
			uint8_t type = data[1];
			uint16_t company_id = (data[3] << 8) | data[2];
			
			if (type == BT_DATA_MANUFACTURER_DATA && company_id == 0x0059) {
				// Parse bitmap (3 bytes starting at offset 4)
				uint32_t bitmap = (data[6] << 16) | (data[5] << 8) | data[4];
				
				// Check if our slot bit is set (indicating we should retransmit)
				if (bitmap & (1 << response_slot)) {
					printk("Synchronizer told to retransmit (slot %d, bitmap=0x%08X)\n", 
					       response_slot, bitmap);
				}
			}
		}
	}
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	printk("BIG INFO report received\n");
}

static void cte_report_cb(struct bt_le_per_adv_sync *sync,
			  struct bt_df_per_adv_sync_iq_samples_report const *report)
{
	printk("CTE report received\n");
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb,
	.cte_report = cte_report_cb,
};

static void response_cb(struct bt_le_per_adv_sync *sync,
			const struct bt_le_per_adv_sync_subevent_info *info,
			struct bt_le_per_adv_sync_response_info *response_info,
			struct net_buf_simple *buf)
{
	int err;

	if (buf) {
		printk("Response transmitted\n");
	}

	if (response_info->tx_status) {
		printk("Response failed to transmit\n");
	}

	if (response_info->subevent != subevent) {
		printk("Subevent mismatch, expected %d, got %d\n", subevent,
		       response_info->subevent);
	}

	if (response_info->response_slot != response_slot) {
		printk("Response slot mismatch, expected %d, got %d\n", response_slot,
		       response_info->response_slot);
	}

	/* Configure the response for the next subevent */
	subevent_params.subevent = subevent;
	subevent_params.response_slot_start = response_slot;
	subevent_params.response_slot_count = 1;
	subevent_params.data_len = PACKET_SIZE;
	subevent_params.data = subevent_data;

	err = bt_le_per_adv_sync_subevent_response(sync, &subevent_params);
	if (err) {
		printk("Failed to set subevent response (err %d)\n", err);
	}
}

static struct bt_le_per_adv_sync_cb subevent_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb,
	.cte_report = cte_report_cb,
	.subevent_data = subevent_data_cb,
	.response = response_cb,
};

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	if (err) {
		return;
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02X %s\n", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

static ssize_t write_pawr_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset,
			       uint8_t flags)
{
	int err;
	struct pawr_timing *timing = (struct pawr_timing *)buf;

	if (len != sizeof(*timing)) {
		printk("Invalid length %d, expected %d\n", len, sizeof(*timing));
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		printk("Invalid offset %d, expected 0\n", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	subevent = timing->subevent;
	response_slot = timing->response_slot;

	printk("Subevent %d, response slot %d\n", subevent, response_slot);

	/* Configure the response for the next subevent */
	subevent_params.subevent = subevent;
	subevent_params.response_slot_start = response_slot;
	subevent_params.response_slot_count = 1;
	subevent_params.data_len = PACKET_SIZE;
	subevent_params.data = subevent_data;

	err = bt_le_per_adv_sync_subevent_response(sync, &subevent_params);
	if (err) {
		printk("Failed to set subevent response (err %d)\n", err);
	}

	return len;
}

#define MAX_BUFFER_SIZE 73  /* Maximum safe buffer size before HCI error */

struct pawr_timing {
	uint8_t subevent;
	uint8_t response_slot;
} __packed;

static void past_received_cb(struct bt_conn *conn,
			     const struct bt_le_per_adv_sync_transfer_received_info *info,
			     struct bt_le_per_adv_sync *per_adv_sync)
{
	printk("PAST received\n");

	sync = per_adv_sync;
	bt_le_per_adv_sync_cb_register(sync, &subevent_callbacks);
}

static struct bt_le_per_adv_sync_transfer_cb past_cb = {
	.received = past_received_cb,
};

void init_subevent_data(void)
{
	/* Initialize subevent data */
	subevent_data[0] = PACKET_SIZE - 1; /* Length */
	subevent_data[1] = BT_DATA_MANUFACTURER_DATA;
	subevent_data[2] = 0x59; /* Nordic */
	subevent_data[3] = 0x00;

	/* Fill remaining bytes with pattern */
	for (int i = 4; i < PACKET_SIZE; i++) {
		subevent_data[i] = i & 0xFF;
	}
}

static char *phy_str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

int main(void)
{
	int err;
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	init_subevent_data();

	printk("Starting PAwR Sync Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* Register the service */
	err = bt_gatt_service_register(&pawr_svc);
	if (err) {
		printk("GATT service register failed (err %d)\n", err);
		return 0;
	}

	/* Initialize advertising data */
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)),
	};

	struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, "PAwR sync sample", 16),
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	/* Register PAST callback */
	bt_le_per_adv_sync_transfer_cb_register(&past_cb);

	printk("Advertising successfully started\n");

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}