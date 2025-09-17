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
#define TARGET_NAME "PAwR_Adv"
//#define MAX_INDIVIDUAL_RESPONSE_SIZE 247 
static uint16_t rsp_size = 232;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync *default_sync;
static bool is_syncing = false;

static struct __packed {
	uint8_t subevent;
	uint8_t response_slot;
	uint16_t total_devices;
} pawr_timing;

static void sync_cb(struct bt_le_per_adv_sync *sync, 
                   struct bt_le_per_adv_sync_synced_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("[SYNC]Synced to %s with %d subevents\n", le_addr, info->num_subevents);

    default_sync = sync;
    is_syncing = true;

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
        } else{
            printk("Successfully sent %d bytes of response data on slot %d\n", rsp_size, pawr_timing.response_slot);
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
        //default_conn = NULL;
        //is_syncing = false;
        return;
    }

    //is_syncing = true;
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
    k_sem_give(&sem_per_adv);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{   
    /*
    bt_conn_unref(default_conn);
    default_conn = NULL;

    if(!default_sync){
        is_syncing = false;
    }*/

    printk("[CONN] Disconnected (reason 0x%02X %s) \n", reason, bt_hci_err_to_str(reason));

    if (default_conn){
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    is_syncing = false;
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* --- Scan parsing helper --- */
static bool name_data_cb(struct bt_data *data, void *user_data)
{
    char *name = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        size_t len = MIN((size_t)data->data_len, NAME_LEN - 1);
        memcpy(name, data->data, len);
        name[len] = '\0';
        return false; /* stop parsing */
    }
    return true;
}

/* --- Device found callback for scanning --- */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *ad)
{
    char name[NAME_LEN] = {0};

    /* interested in connectable adv only */
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND && adv_type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    bt_data_parse(ad, name_data_cb, name);
    if (name[0] == '\0') {
        return;
    }

    if (strcmp(name, TARGET_NAME) != 0) {
        return;
    }

    //printk("[SCAN] Found target '%s' at %s, connecting...\n", name, bt_addr_to_str(addr));

    /* stop scanning and connect */
    bt_le_scan_stop();

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &default_conn);
    if (err) {
        printk("[SCAN] bt_conn_le_create failed (err %d)\n", err);
        default_conn = NULL;
    } 
}
/*
static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};*/

int main(void)
{
	int err;

	printk("Starting PAwR Synchronizer\n");

	err = bt_enable(NULL);
	if (err) {
		printk("[INIT] Error: Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_le_per_adv_sync_cb_register(&sync_callbacks);


    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_PASSIVE, 
        .options = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE, 
        .interval = 0x0060,
        .window = 0x0030,
    };

    err = bt_le_scan_start(&scan_param, device_found);
    if (err){
        printk("[Scan] Failed to start scanning (err %d)\n", err);
        return 0;
    }
    printk("[SCAN] Scanning for '%s'...\n", TARGET_NAME);

     /* wait for connect signal from connected() */
     if (k_sem_take(&sem_per_adv, K_SECONDS(20)) != 0) {
        printk("[MAIN] Timeout waiting for connection (no advertiser found)\n");
        return 0;
    }

    struct bt_le_per_adv_sync_transfer_param past_param = {
        .skip = 0,
        .timeout = 3000, /* 3000 * 10ms = 30 s - generous for debugging */
        .options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE,
    };

    printk("[PAST] Subscribing for PAST (conn %p)\n", default_conn);

	err = bt_le_per_adv_sync_transfer_subscribe(default_conn, &past_param);  //change Null to something else??
	if (err) {
		printk("[PAST] bt_le_per_adv_sync_transfer_subscribe failed (err %d)\n", err);
		return 0;
	} else {
        printk("[PAST] Subscribe returned 0, waiting for synced callback\n");
    }
    
    if(k_sem_take(&sem_per_sync, K_SECONDS(35)) != 0){
        printk("[MAIN] Synced callback timed out (waiting for PAST or fallback sync)\n");
     } else {
        printk("[MAIN] Synced! enabling periodic receive\n");
        if (default_sync) {
            err = bt_le_per_adv_sync_recv_enable(default_sync);
            if (err) {
                printk("[MAIN] bt_le_per_adv_sync_recv_enable failed (err %d)\n", err);
            } else {
                printk("[MAIN] Periodic recv enabled\n");
            }
        }
    }
    
	do {
         /* If sync terminates, sem_per_sync_lost will be given */
         if (k_sem_take(&sem_per_sync_lost, K_SECONDS(1)) == 0) {
            printk("[MAIN] Sync lost; attempting to re-scan\n");

            /* cleanup connection (if present) */
            if (default_conn) {
                bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                bt_conn_unref(default_conn);
                default_conn = NULL;
            }
            is_syncing = false;
            default_sync = NULL;

            /* restart scanning */
            err = bt_le_scan_start(&scan_param, device_found);
            if (err) {
                printk("[MAIN] Failed to restart scanning (err %d)\n", err);
            }
            /* wait for next connect... */
            if (k_sem_take(&sem_per_adv, K_SECONDS(20)) != 0) {
                printk("[MAIN] No advertiser found during re-scan timeout\n");
            } else {
                /* subscribe again to PAST (repeat above flow) */
                printk("[MAIN] Reconnected; subscribe for PAST again\n");
                err = bt_le_per_adv_sync_transfer_subscribe(default_conn, &past_param);
                if (err) {
                    printk("[MAIN] PAST subscribe failed on reconnect (err %d)\n", err);
                }
                /* wait for sync as before... */
                (void)k_sem_take(&sem_per_sync, K_SECONDS(35));
                if (default_sync) {
                    bt_le_per_adv_sync_recv_enable(default_sync);
                }
            }
        }
    k_sleep(K_MSEC(100));
	} while (true);

	return 0;
}
