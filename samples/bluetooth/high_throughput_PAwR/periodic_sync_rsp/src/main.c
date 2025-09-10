/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>

#define NAME_LEN 30
#define MAX_INDIVIDUAL_RESPONSE_SIZE 247 
static uint16_t rsp_size = 230;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync *default_sync;
static bool is_syncing;

static struct __packed {
	uint8_t subevent;
	uint8_t response_slot;
	uint16_t total_devices;
} pawr_timing;

static void sync_cb(struct bt_le_per_adv_sync *sync, 
                   struct bt_le_per_adv_sync_synced_info *info)
{
    //struct bt_le_per_adv_sync_subevent_params params;
    //uint8_t subevents[1];
    char le_addr[BT_ADDR_LE_STR_LEN];
    //int err;

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("[SYNC]Synced to %s with %d subevents\n", le_addr, info->num_subevents);

    default_sync = sync;

    /*
    params.properties = 0;
    params.num_subevents = 1;
    params.subevents = subevents;
    subevents[0] = pawr_timing.subevent;

    err = bt_le_per_adv_sync_subevent(sync, &params);
    if (err) {
        printk("[SYNC]Failed to set subevents (err %d)\n", err);
    } else {
        printk("[SYNC] Changed sync to subevent %d\n", subevents[0]);
    }
    */
    k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                   const struct bt_le_per_adv_sync_term_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));


    default_sync = NULL;
    is_syncing = false;
    k_sem_give(&sem_per_sync_lost);
}



static struct bt_le_per_adv_response_params rsp_params;

NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, 260);

static void recv_cb(struct bt_le_per_adv_sync *sync,
                   const struct bt_le_per_adv_sync_recv_info *info,
                   struct net_buf_simple *buf)
{
    int err;

    if (buf && buf->len) {
        /* Parse manufacturer data to extract retransmission bitmap */
        if (buf->len >= 7) {  
            uint8_t *data = buf->data;
            
            // Check if this is manufacturer data with Nordic Company ID
            if (data[0] >= 6 &&  
                data[1] == BT_DATA_MANUFACTURER_DATA &&
                data[2] == 0x59 && data[3] == 0x00) {  // Nordic Company ID (little endian)
                
                uint8_t bitmap_bytes = data[0] - 5;  
                
                uint32_t retransmit_bitmap = 0;
                for (int i = 0; i < bitmap_bytes && i < 4; i++) { 
                    retransmit_bitmap |= (uint32_t)data[4 + i] << (i * 8);
                }
                
                if (pawr_timing.response_slot < (bitmap_bytes * 8) && 
                    (retransmit_bitmap & (1 << pawr_timing.response_slot))) {
                    printk("RETRANSMIT: Slot %d told to retransmit (bitmap=0x%08X, %d bytes)\n", 
                           pawr_timing.response_slot, retransmit_bitmap, bitmap_bytes);
                }
            }
        }
        
        /* Generate raw test data using dynamic response size */
        net_buf_simple_reset(&rsp_buf);
        
        for (int i = 0; i < rsp_size; i++) {
            net_buf_simple_add_u8(&rsp_buf, pawr_timing.response_slot + i);
        }

        rsp_params.request_event = info->periodic_event_counter;
        rsp_params.request_subevent = info->subevent;
        rsp_params.response_subevent = info->subevent;
        rsp_params.response_slot = pawr_timing.response_slot;

        err = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);
        if (err) {
            printk("Failed to send response (err %d)\n", err);
        }
    } else {
        printk("Failed to receive on subevent %d\n", info->subevent);
    }
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
};

static const struct bt_uuid_128 pawr_svc_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));
static const struct bt_uuid_128 pawr_char_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static ssize_t write_timing(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(pawr_timing)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&pawr_timing, buf, len);

	struct bt_le_per_adv_sync_subevent_params params;
	uint8_t subevents[1];
	int err;

	params.properties = 0;
	params.num_subevents = 1;
	params.subevents = subevents;
	subevents[0] = pawr_timing.subevent;

	if (default_sync) {
        k_sleep(K_MSEC(10));
		err = bt_le_per_adv_sync_subevent(default_sync, &params);
		if (err) {
			printk("[TIMING] Error: Failed to set subevents (err %d)\n", err);
		}
	}

	return len;
}

BT_GATT_SERVICE_DEFINE(pawr_svc, BT_GATT_PRIMARY_SERVICE(&pawr_svc_uuid.uuid),
		       BT_GATT_CHARACTERISTIC(&pawr_char_uuid.uuid, BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_WRITE, NULL, write_timing,
					      &pawr_timing));

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("[SYNC]Failed to connect (err 0x%02X)\n", err);
        default_conn = NULL;
        is_syncing = false;
        return;
    }

    is_syncing = true;
    default_conn = bt_conn_ref(conn);

    /* Accept 2M PHY if requested */
    struct bt_conn_le_phy_param phy_param = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    
    err = bt_conn_le_phy_update(conn, &phy_param);
    if (err) {
        printk("[SYNC] PHY update request failed (err %d)\n", err);
    } else {
        printk("[SYNC] PHY update request sent\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    bt_conn_unref(default_conn);
    default_conn = NULL;

    if(!default_sync){
        is_syncing = false;
    }
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int main(void)
{
	struct bt_le_per_adv_sync_transfer_param past_param;
	int err;

	printk("Starting PAwR Synchronizer\n");

	err = bt_enable(NULL);
	if (err) {
		printk("[INIT] Error: Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	past_param.skip = 1;
	past_param.timeout = 1000; /* 10 seconds */
	past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE;
	err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
	if (err) {
		printk("[INIT] Error: PAST subscribe failed (err %d)\n", err);
		return 0;
	}

	do {
        if(is_syncing && !default_conn && !default_sync){
            is_syncing = false;
        }
        
        if(!is_syncing){
            bt_le_adv_stop();

            err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
            if (err){
                if (err == -EALREADY) {
                k_sleep(K_MSEC(10));
                continue;
                } else if(err == -ENOMEM){
                    k_sleep(K_MSEC(500));
                    continue;
                } else{
                    printk("[ADV] Error: Failed to start (err %d)\n", err);
                    k_sleep(K_MSEC(100));
                    continue;
                }
            } 
        }
        
		printk("[SYNC] Waiting for sync...\n");

		err = k_sem_take(&sem_per_sync, K_SECONDS(10));
		if (err) {
			printk("[SYNC] Error: Timeout while syncing\n");
            is_syncing = false; 
            k_sleep(K_MSEC(100));
			continue;
		}

		err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		if (err) {
			printk("[SYNC] Error: Failed (err %d)\n", err);
			return 0;
		}
	} while (true);

	return 0;
}
