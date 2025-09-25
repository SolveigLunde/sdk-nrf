/*
* Copyright (c) 2023 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: Apache-2.0
*/

// -----------------------Synchroniser code ---------------------------
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
 
#define NAME_LEN 30
#ifndef BT_LE_PER_ADV_SYNC_SUBEVENT_PARAMS_ENABLE_RESPONSE
#define BT_LE_PER_ADV_SYNC_SUBEVENT_PARAMS_ENABLE_RESPONSE BIT(0)
#endif
/* Ensure we request automatic sync creation on PAST receive (if available) NEW*/  
#ifndef BT_LE_PER_ADV_SYNC_TRANSFER_OPT_SYNC
#define BT_LE_PER_ADV_SYNC_TRANSFER_OPT_SYNC BIT(0)
#endif
static uint16_t rsp_size = 16; //232

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync *default_sync;
static bool is_syncing;  // Track if we're in the process of syncing
static bt_addr_le_t last_peer_addr; /* advertiser's address captured from GATT conn */
static uint8_t last_peer_sid = 0;   /* assume SID 0 unless set explicitly */
static bool last_peer_addr_valid;

static struct __packed {
    uint8_t subevent;
    uint8_t response_slot;
    uint16_t total_devices;
    uint8_t adv_sid;
} pawr_timing;

static bool have_pending_filter;
static uint8_t pending_subevent;
static bool scanning_started;

static void apply_subevent_filter_if_ready(void)
{
    if (!default_sync || !have_pending_filter) {
        return;
    }

    struct bt_le_per_adv_sync_subevent_params params;
    uint8_t subevents[1] = {pending_subevent};
    params.properties   = BT_LE_PER_ADV_SYNC_SUBEVENT_PARAMS_ENABLE_RESPONSE;
    params.num_subevents = 1;
    params.subevents     = subevents;
    //subevents[0]         = pending_subevent;

    if(have_pending_filter){
        int err = bt_le_per_adv_sync_subevent(default_sync, &params);
    if (err) {
        printk("[TIMING] set subevents failed (err %d)\n", err);
    } else {
        printk("[TIMING] subevent filter applied: %u\n", pending_subevent);
        have_pending_filter = false; /* done */
    }
    }
}


static void sync_cb(struct bt_le_per_adv_sync *sync,
    struct bt_le_per_adv_sync_synced_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    printk("[SYNC] Synced cb (sync=%p) to %s: sid=%u, interval=%u*1.25ms, subevents=%u, addr_type=%u\n",
            sync, le_addr, info->sid, info->interval, info->num_subevents, info->addr->type);

    /* Verify the sync parameters match what we expect */
    if (pawr_timing.total_devices > 1) {
        /* We're in aggressive mode after re-PAST */
        uint16_t expected_interval = (uint16_t)(1 + (9u * pawr_timing.total_devices + 9u) / 10u);
        expected_interval += 1; /* slight pad, matching advertiser */
        
        if (info->interval != expected_interval) {
            printk("[SYNC] Warning: Unexpected interval %u (expected %u for %u devices)\n",
                   info->interval, expected_interval, pawr_timing.total_devices);
            /* Don't fail - just warn. The advertiser might be using different params */
        } else {
            printk("[SYNC] Verified aggressive mode parameters (interval=%u for %u devices)\n",
                   info->interval, pawr_timing.total_devices);
        }
    }

    default_sync = sync;

    /* Now we have a handle → apply any pending subevent filter */
    apply_subevent_filter_if_ready();

    k_sem_give(&sem_per_sync);
    printk("OMGGGGGGG GAVE sem_per_sync\n");  
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                   const struct bt_le_per_adv_sync_term_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    default_sync = NULL;
    is_syncing = false;  // Reset syncing state when sync is terminated

    k_sem_give(&sem_per_sync_lost);
}



static struct bt_le_per_adv_response_params rsp_params;

NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, 260);

static void recv_cb(struct bt_le_per_adv_sync *sync,
                const struct bt_le_per_adv_sync_recv_info *info,
                struct net_buf_simple *buf)
{   
    //if (info) {
    //    printk("[RECV] period_event=%u subevent=%u sid=%u rssi=%d\n",
    //           info->periodic_event_counter, info->subevent, info->sid, info->rssi);
    //}

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
        } else {
            printk("Successfully sent %d bytes of response data on slot %d\n", rsp_size, pawr_timing.response_slot);
        }
    } else {
        //printk("Failed to receive on subevent %d\n", info->subevent);
    }
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .recv = recv_cb,
};

/* Extended scanning callback to learn advertiser SID for explicit sync create */
static void scan_recv_cb(const struct bt_le_scan_recv_info *info,
                        struct net_buf_simple *buf)
{
    ARG_UNUSED(buf);
    if (!info || !info->addr) {
        return;
    }

    /* Ignore reports without a valid SID (0xFF means invalid) */
    if (info->sid == 0xFF) {
        return;
    }

    if (!last_peer_addr_valid) {
        bt_addr_le_copy(&last_peer_addr, info->addr);
        last_peer_addr_valid = true;
        last_peer_sid = info->sid;
        char a[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&last_peer_addr, a, sizeof(a));
        printk("[SCAN] learned advertiser %s sid=%u\n", a, last_peer_sid);
        /* If we are waiting to sync and have no current sync, try create now */
        if (pawr_timing.total_devices > 0 && default_sync == NULL && last_peer_sid <= 0x0F) {
            struct bt_le_per_adv_sync_param sp = {0};
            sp.addr = last_peer_addr;
            sp.sid = last_peer_sid;
            sp.skip = 0;
            sp.timeout = 2000;
            sp.options = 0;
            int err = bt_le_per_adv_sync_create(&sp, &default_sync);
            if (err) {
                printk("[SCAN] explicit sync create (on scan) failed (err %d)\n", err);
            } else {
                printk("[SCAN] explicit sync create requested (on scan)\n");
            }
        }
        return;
    }

    if (bt_addr_le_cmp(&last_peer_addr, info->addr) == 0) {
        /* refresh sid if it changes */
        last_peer_sid = info->sid;
        if (pawr_timing.total_devices > 0 && default_sync == NULL && last_peer_sid <= 0x0F) {
            struct bt_le_per_adv_sync_param sp = {0};
            sp.addr = last_peer_addr;
            sp.sid = last_peer_sid;
            sp.skip = 0;
            sp.timeout = 2000;
            sp.options = 0;
            int err = bt_le_per_adv_sync_create(&sp, &default_sync);
            if (err) {
                printk("[SCAN] explicit sync create (on scan refresh) failed (err %d)\n", err);
            } else {
                printk("[SCAN] explicit sync create requested (on scan refresh)\n");
            }
        }
    }
}

static struct bt_le_scan_cb scan_cbs = {
    .recv = scan_recv_cb,
};

/* Note: explicit PAST receive callbacks are not available in this SDK tree. */

static const struct bt_uuid_128 pawr_svc_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));
static const struct bt_uuid_128 pawr_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static ssize_t write_timing(struct bt_conn *conn, const struct bt_gatt_attr *attr,
        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(pawr_timing)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&pawr_timing, buf, len);

    /* Just cache the subevent. We’ll apply it once we *have* a sync handle. */
    pending_subevent     = pawr_timing.subevent;
    have_pending_filter  = true;

    /* If we already synced (e.g., re-PAST path), apply now. */
    apply_subevent_filter_if_ready();

    printk("[TIMING] got timing: subevent=%u, slot=%u, total=%u (sid=%u)\n",
    pawr_timing.subevent, pawr_timing.response_slot, pawr_timing.total_devices, pawr_timing.adv_sid);

    /* Stop connectable advertising after provisioning to favor scanning+sync */
    (void)bt_le_adv_stop();

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

    /* Do NOT overwrite the periodic advertiser address learned from scan.
     * Only cache connection peer addr if we don't have a periodic adv addr yet. */
    struct bt_conn_info cinfo;
    if (bt_conn_get_info(conn, &cinfo) == 0 && cinfo.type == BT_CONN_TYPE_LE) {
        if (!last_peer_addr_valid) {
            bt_addr_le_copy(&last_peer_addr, cinfo.le.dst);
            last_peer_addr_valid = true;
            char a[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&last_peer_addr, a, sizeof(a));
            printk("[SYNC] cached peer addr %s for explicit sync create (sid=%u)\n", a, last_peer_sid);
        }
    }

    /* Accept 2M PHY if requested */
    struct bt_conn_le_phy_param phy_param = {
        .options   = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    int perr = bt_conn_le_phy_update(conn, &phy_param);
    if (perr) {
        printk("[SYNC] PHY update request failed (err %d)\n", perr);
    } else {
        printk("[SYNC] PHY update request sent\n");
    }

    /* Ensure PAST is enabled for THIS connection (required to accept re-PAST) */
    struct bt_le_per_adv_sync_transfer_param past_param = {
        .skip    = 0,
        .timeout = 2000, /* 20.00 s (10 ms units) */
        .options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_SYNC,
    };
    int serr = bt_le_per_adv_sync_transfer_subscribe(conn, &past_param);
    if (serr) {
        printk("[PAST] per-conn subscribe failed (err %d)\n", serr);
    } else {
        printk("[PAST] per-conn subscribe OK\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    /* If we already have a slot assignment, force re-sync to latch new periodic */
    if (pawr_timing.total_devices > 0) {
        if (default_sync) {
            int derr = bt_le_per_adv_sync_delete(default_sync);
            if (derr) {
                printk("[SYNC] forced delete of periodic sync failed (err %d)\n", derr);
            } else {
                printk("[SYNC] forced delete of periodic sync OK\n");
            }
            default_sync = NULL;
        }
        /* Stay in syncing mode; passive scan already enabled; wait for new synced cb */
        is_syncing = true;
        k_sem_reset(&sem_per_sync);
        printk("[SYNC] Forcing re-sync after GATT disconnect\n");
        /* Do not create sync here; wait for scan with valid SID */
    } else {
        if (!default_sync) {
            is_syncing = false;   /* let main loop try again */
        }
    }
    printk("[SYNC] Disconnected (0x%02X)\n", reason);
}



BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Passive scan callback (unused, but required by API) */
static void noop_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad)
{
    ARG_UNUSED(addr);
    ARG_UNUSED(rssi);
    ARG_UNUSED(type);
    ARG_UNUSED(ad);
}

int main(void)
{
    int err;

    printk("Starting PAwR Synchronizer\n");

    err = bt_enable(NULL);
    if (err) {
        printk("[INIT] Error: Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    /* Register scan callbacks to learn advertiser SID/address */
    bt_le_scan_cb_register(&scan_cbs);

    bt_le_per_adv_sync_cb_register(&sync_callbacks);
    printk("[INIT] Per-adv sync callbacks registered\n");

    struct bt_le_per_adv_sync_transfer_param past_param = {
        .skip    = 0,       /* don’t skip PA reports */
        .timeout = 2000,    /* 20.00 s (units are 10 ms) */
        .options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_SYNC,
    };
    err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
    if (err) {
        printk("[INIT] PAST global subscribe failed (err %d)\n", err);
        return 0;
    }
    printk("[INIT] PAST global subscribe OK\n");

    /* No explicit PAST receive callback API in this SDK; rely on OPT_SYNC. */

    /* Keep passive scanning enabled so controller can latch periodic after PAST */
    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, noop_scan_cb);
    if (err && err != -EALREADY) {
        printk("[INIT] Passive scan start failed (err %d)\n", err);
    } else {
        scanning_started = true;
        printk("[INIT] Passive scan enabled\n");
    }

    /* Register PAwR sync lifecycle callbacks (these you already have) */
    //bt_le_per_adv_sync_cb_register(&sync_callbacks);

    /* NOTE: do NOT call bt_le_per_adv_sync_transfer_cb_register();
       that API is not present in your tree and isn't needed. */

    do {
        if (is_syncing && !default_conn && !default_sync) {
            is_syncing = false;
        }

        if (!is_syncing) {
            (void)bt_le_adv_stop();

            err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
            if (err) {
                if (err == -EALREADY) {
                    k_sleep(K_MSEC(10));
                    continue;
                } else if (err == -ENOMEM) {
                    k_sleep(K_MSEC(500));
                    continue;
                } else {
                    printk("[ADV] Error: Failed to start (err %d)\n", err);
                    k_sleep(K_MSEC(100));
                    continue;
                }
            }
        }

        printk("[SYNC] Waiting for sync...\n");
        err = k_sem_take(&sem_per_sync, K_SECONDS(20));  // was 10; make it 20 initially
        if (err) {
            printk("[SYNC] sem_per_sync timed out after 20s\n");  
            /* After provisioning (slot assigned), prefer staying in syncing mode */
            if (pawr_timing.total_devices > 0) {
                is_syncing = true;
            } else {
                is_syncing = false;
            }
            k_sleep(K_MSEC(100));
            continue;
        }


        /* Stay in data phase until sync is lost/terminated */
        err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
        printk("[SYNC] Data phase, have locked semaphore\n");
        if (err) {
            printk("[SYNC] Error: Failed (err %d)\n", err);
            return 0;
        }
    } while (true);

    return 0;
}