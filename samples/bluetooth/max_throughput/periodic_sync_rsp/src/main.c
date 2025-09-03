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

// Buffer size constraints from Nordic SDK
#define MAX_PAWR_TOTAL_BUFFER_SIZE 1650  // CONFIG_BT_CTLR_ADV_DATA_LEN_MAX limit
#define MAX_INDIVIDUAL_RESPONSE_SIZE 247 // BLE spec limit for individual responses

// Dynamic device count - will be set by advertiser
static uint8_t actual_device_count = 0;
static uint16_t dynamic_rsp_size = 0;

// Onboarding phase detection
static bool advertiser_in_onboarding = true;
static uint16_t onboarding_response_size = 5; // Minimal response size during onboarding


static uint16_t calculate_optimal_response_size(uint8_t device_count) {
    if (device_count == 0) {
        device_count = 16; // Fallback only if not set
    }
    uint16_t optimal_size = (MAX_PAWR_TOTAL_BUFFER_SIZE - 20) / device_count;
    if (optimal_size > MAX_INDIVIDUAL_RESPONSE_SIZE) {
        optimal_size = MAX_INDIVIDUAL_RESPONSE_SIZE;
    }
    
    // Optional: Reduce size slightly to minimize RF collisions while maintaining good throughput
    // Uncomment next line if you want to prioritize fewer collisions over maximum throughput
    // optimal_size = (optimal_size * 80) / 100;  // Use 80% of optimal for better reliability
    
    return optimal_size;
}

//static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync *default_sync;

static struct __packed {
    uint8_t subevent;
    uint8_t response_slot;
    uint8_t total_devices;  
} pawr_timing;

static void sync_cb(struct bt_le_per_adv_sync *sync, 
                struct bt_le_per_adv_sync_synced_info *info)
{
    struct bt_le_per_adv_sync_subevent_params params;
    uint8_t subevents[1];
    char le_addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("[SYNC]Synced to %s with %d subevents\n", le_addr, info->num_subevents);

    default_sync = sync;

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

    k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                const struct bt_le_per_adv_sync_term_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));


    default_sync = NULL;

    k_sem_give(&sem_per_sync_lost);
}



static struct bt_le_per_adv_response_params rsp_params;

NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, 260);

static void recv_cb(struct bt_le_per_adv_sync *sync,
                const struct bt_le_per_adv_sync_recv_info *info,
                struct net_buf_simple *buf)
{
    int err;

    // Initialize dynamic response size if not set
    if (dynamic_rsp_size == 0) {
        dynamic_rsp_size = calculate_optimal_response_size(actual_device_count);
        if (actual_device_count > 0) {
            printk("📱 Responder using dynamic response size: %d bytes\n", dynamic_rsp_size);
            printk("   Configured for %d devices = %d bytes total\n", 
                actual_device_count, actual_device_count * dynamic_rsp_size);
        } else {
            printk("📱 Responder using fallback response size: %d bytes\n", dynamic_rsp_size);
            printk("   Will be reconfigured when advertiser provides device count\n");
        }
    }

    if (buf && buf->len) {
        /* Parse manufacturer data to detect onboarding vs measurement phase */
        bool was_onboarding = advertiser_in_onboarding;
        
        if (buf->len >= 6) {  // Minimum length for our format
            uint8_t *data = buf->data;
            
            // Debug output removed for cleaner logs
            
            if (data[0] >= 5 &&  
                data[1] == BT_DATA_MANUFACTURER_DATA &&
                data[2] == 0x59 && data[3] == 0x00) {  // Nordic Company ID
                
                // Check if this is onboarding phase vs measurement phase
                // Onboarding: exactly 5 bytes [05 FF 59 00 00]  
                // Measurement: 5+ bytes [05+ FF 59 00 XX XX...] with bitmap data
                if (data[0] == 4 && buf->len == 5) {
                    // Onboarding phase: length=4, total 5 bytes, single 0x00 marker
                    advertiser_in_onboarding = true;
                    if (!was_onboarding) {
                        printk("[SYNC] Detected advertiser returned to onboarding phase\n");
                    }
                } else if (data[0] >= 5 && buf->len >= 6) {
                    // Measurement phase: length>=5, total 6+ bytes, has bitmap data
                    if (advertiser_in_onboarding) {
                        printk("[SYNC] *** ADVERTISER READY - SWITCHING TO FULL DATA MODE ***\n");
                        advertiser_in_onboarding = false;
                    }
                }
            }
                
                // Calculate bitmap size from packet length
                uint8_t bitmap_bytes = data[0] - 5; 
                
                // Extract retransmission bitmap
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
        
        /* Generate response data based on current phase */
        net_buf_simple_reset(&rsp_buf);
        
        uint16_t response_size;
        if (advertiser_in_onboarding) {
            // Onboarding phase: send minimal response to confirm sync
            response_size = onboarding_response_size;
        } else {
            // Measurement phase: send full-size response for throughput testing
            response_size = dynamic_rsp_size;
        }
        
        // Fill the buffer with raw bytes - no headers, no structure
        for (int i = 0; i < response_size; i++) {
            net_buf_simple_add_u8(&rsp_buf, pawr_timing.response_slot + i);
        }

        rsp_params.request_event = info->periodic_event_counter;
        rsp_params.request_subevent = info->subevent;
        rsp_params.response_subevent = info->subevent;
        rsp_params.response_slot = pawr_timing.response_slot;

        if (advertiser_in_onboarding) {
            printk("[SYNC] Onboarding response: subevent %d, slot %d, %d bytes\n", 
                info->subevent, pawr_timing.response_slot, rsp_buf.len);
        } else {
        printk("Indication: subevent %d, responding in slot %d with %d bytes\n", 
            info->subevent, pawr_timing.response_slot, rsp_buf.len);
        }

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

    // Update actual device count and recalculate response size
    actual_device_count = pawr_timing.total_devices;
    dynamic_rsp_size = calculate_optimal_response_size(actual_device_count);
    
    printk("[SYNC] Configured: slot %d, total devices %d, response size %d bytes\n",
        pawr_timing.response_slot, actual_device_count, dynamic_rsp_size);


    struct bt_le_per_adv_sync_subevent_params params;
    uint8_t subevents[1];
    int err;

    params.properties = 0;
    params.num_subevents = 1;
    params.subevents = subevents;
    subevents[0] = pawr_timing.subevent;

    if (default_sync) {
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
        return;
    }

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
        err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err && err != -EALREADY) {
            printk("[ADV] Error: Failed to start (err %d)\n", err);
            return 0;
        }

        printk("[SYNC] Waiting for sync...\n");
        err = k_sem_take(&sem_per_sync, K_SECONDS(10));
        if (err) {
            printk("[SYNC] Error: Timeout while syncing\n");
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