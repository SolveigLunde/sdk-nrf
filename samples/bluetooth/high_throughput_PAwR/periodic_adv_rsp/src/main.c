/*
* Copyright (c) 2023 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: Apache-2.0
*/

// ---------------------Advertisier code-----------------------------
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

#define SYNC_RSP_SIZE 232  /* payload size each device responds with */
#define THROUGHPUT_PRINT_INTERVAL 1000
#define UNIT_MS 1.25

static uint32_t total_bytes;
static uint64_t stamp;

static uint16_t g_active_num_slots = 1; /* how many response slots are *currently* active */
static uint8_t  g_active_bitmap_bytes = 1; /* ceil(g_active_num_slots / 8) */

/* bitmap helper must be defined before MAX_* macros use it */
#define BITMAP_BYTES_NEEDED(num_devices) (((num_devices) + 7) / 8)

/* Bitmaps sized to the *maximum* you’ll ever use (compile-time cap) */
#define MAX_DEVICES       CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define MAX_BITMAP_BYTES  BITMAP_BYTES_NEEDED(MAX_DEVICES)

/* Reuse your existing arrays but size them to MAX_DEVICES */
static uint8_t response_bitmap[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];
static uint8_t expected_responses[BITMAP_BYTES_NEEDED(CONFIG_BT_MAX_THROUGHPUT_DEVICES)];

typedef struct {
    uint8_t  num_devices;
    uint16_t packet_size;
    uint16_t total_bytes_per_interval;
    uint16_t interval_ms;
} config_t;

/* ---------- Lenient PAwR for provisioning (keeps duty low so connects/GATT aren't starved) ---------- */
static void set_pawr_params_lenient(struct bt_le_per_adv_param *p) {
    /* 120 * 1.25ms = 150ms periodic; 1 slot only */
    p->interval_min          = p->interval_max = 120;
    p->num_subevents         = 1;
    p->subevent_interval     = 120;
    p->response_slot_delay   = 1;   /* 1.25ms */
    p->response_slot_spacing = 20;  /* 2.5ms */
    p->num_response_slots    = 1;   /* IMPORTANT: only 1 during Phase A */
}

/* ---------- Aggressive PAwR for data phase ---------- */
void set_pawr_params(struct bt_le_per_adv_param *params, uint8_t num_response_slots) {
    const uint8_t RSP_DELAY    = 1;  /* 1.25ms */
    const uint8_t SLOT_SPACING = 9;  /* 1.125ms (9 * 0.125ms) */

    /* Convert needed time to 1.25ms units */
    uint16_t min_units      = (uint16_t)(1 + (9u * num_response_slots + 9u) / 10u);
    uint16_t interval_units = (min_units < 6 ? 6 : min_units);

    uint32_t total_ms_x100 = (uint32_t)interval_units * 125; /* x100 ms for pretty print */

    interval_units += 1; /* slight pad */

    params->interval_min          = interval_units;
    params->interval_max          = interval_units;
    params->num_subevents         = 1;
    params->subevent_interval     = interval_units;
    params->response_slot_delay   = RSP_DELAY;
    params->response_slot_spacing = SLOT_SPACING;
    params->num_response_slots    = num_response_slots;

    uint32_t throughput_kbps =
        ((uint32_t)num_response_slots * (uint32_t)SYNC_RSP_SIZE * 8u * 1000u)
        / ((uint32_t)interval_units * 1250u);

    printk("PAwR Config for %d devices:\n", num_response_slots);
    printk("  Packet Size: %d bytes\n", SYNC_RSP_SIZE);
    printk("  Response Slot: 1.125 ms (9 * 0.125ms)\n");
    printk("  Initial Delay: 1.25 ms (1 * 1.25ms)\n");
    printk("  Total Time: %u.%02u ms (%u * 1.25ms)\n",
           total_ms_x100 / 100, total_ms_x100 % 100, (unsigned)interval_units);
    printk("  Est. Throughput: %u kbps\n", throughput_kbps);
}

static struct bt_le_per_adv_param per_adv_params;
void init_adv_params(void) {
    set_pawr_params(&per_adv_params, NUM_RSP_SLOTS);
}

/* ---------- PAwR data buffers ---------- */
static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

/* ---------- Bitmap helpers ---------- */
static void bitmap_set_bit(uint8_t *bitmap, int bit) {
    bitmap[bit / 8] |= (1 << (bit % 8));
}
static bool bitmap_test_bit(const uint8_t *bitmap, int bit) {
    return (bitmap[bit / 8] & (1 << (bit % 8))) != 0;
}
static void bitmap_clear(uint8_t *bitmap, int num_bits) {
    memset(bitmap, 0, BITMAP_BYTES_NEEDED(num_bits));
}
static inline void reset_bitmaps_for(uint16_t num_slots) {
    bitmap_clear(response_bitmap,  num_slots);
    bitmap_clear(expected_responses, num_slots);
}
static inline void reset_receipts_for_event(uint16_t num_slots) {
    bitmap_clear(response_bitmap, num_slots); // only per-event receipts
}

/* ---------- Advertiser callbacks ---------- */
static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
    if (!request) { printk("Error: NULL request received\n"); return; }

    uint8_t to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

    /* Build retransmit bitmap sized to *current* slot count */
    uint8_t retransmit_bitmap[MAX_BITMAP_BYTES] = {0};
    for (int i = 0; i < g_active_bitmap_bytes; i++) {
        retransmit_bitmap[i] = expected_responses[i] & (uint8_t)(~response_bitmap[i]);
    }

    bool has_retx = false;
    for (int i = 0; i < g_active_bitmap_bytes; i++) {
        if (retransmit_bitmap[i]) { has_retx = true; break; }
    }
    if (has_retx) {
        printk("Empty response slots detected (active=%u):\n", g_active_num_slots);
        for (int slot = 0; slot < g_active_num_slots; slot++) {
            if (bitmap_test_bit(retransmit_bitmap, slot)) {
                printk("  Slot %d: packet lost\n", slot);
            }
        }
    }

    /* NEW */
    reset_receipts_for_event(g_active_num_slots);

    for (size_t i = 0; i < to_send; i++) {
        struct net_buf_simple *buf = &bufs[i];
        net_buf_simple_reset(buf);

        /* Manufacturer specific data: just the active bitmap bytes */
        uint8_t *length_field = net_buf_simple_add(buf, 1);
        net_buf_simple_add_u8(buf, BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(buf, 0x0059); /* Nordic Company ID */
        for (int j = 0; j < g_active_bitmap_bytes; j++) {
            net_buf_simple_add_u8(buf, retransmit_bitmap[j]);
        }
        *length_field = buf->len - 1;

        subevent_data_params[i].subevent             = request->start + i;
        subevent_data_params[i].response_slot_start  = 0;
        subevent_data_params[i].response_slot_count  = g_active_num_slots;
        subevent_data_params[i].data                 = buf;
    }

    int err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk("Failed to set subevent data (err %d), active=%u\n", err, g_active_num_slots);
    }
}

static void response_cb(struct bt_le_ext_adv *adv,
                        struct bt_le_per_adv_response_info *info,
                        struct net_buf_simple *buf)
{
    if (!buf) return;

    if (total_bytes == 0) {
        stamp = k_uptime_get_32();
    }
    total_bytes += buf->len;

    if (info->response_slot < g_active_num_slots) {
        bitmap_set_bit(response_bitmap, info->response_slot);
        if (!bitmap_test_bit(expected_responses, info->response_slot)) {
            bitmap_set_bit(expected_responses, info->response_slot);
            printk("Slot %d now actively responding\n", info->response_slot);
        }
    }

    if (k_uptime_get_32() - stamp > THROUGHPUT_PRINT_INTERVAL) {
        int64_t delta = k_uptime_delta(&stamp);
        printk("\n[PAwR] received %u bytes (%u KB) in %lld ms at %llu kbps\n",
               total_bytes, total_bytes / 1024, delta,
               ((uint64_t)total_bytes * 8 / (uint64_t)delta));
        total_bytes = 0;
    }
}

static const struct bt_le_ext_adv_cb adv_cb = {
    .pawr_data_request = request_cb,
    .pawr_response     = response_cb,
};

/* ---------- Provisioning sync/connect plumbing ---------- */
static K_SEM_DEFINE(sem_connected,    0, 1);
static K_SEM_DEFINE(sem_discovered,   0, 1);
static K_SEM_DEFINE(sem_written,      0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

static inline void reset_prov_sems(void) {
    k_sem_reset(&sem_connected);
    k_sem_reset(&sem_discovered);
    k_sem_reset(&sem_written);
    k_sem_reset(&sem_disconnected);
}

static struct bt_conn *default_conn;

void connected_cb(struct bt_conn *conn, uint8_t err)
{
    printk("[DEBUG] Connected callback called (err 0x%02X)\n", err);
    //__ASSERT(conn == default_conn, "Unexpected connected callback");

    if (err) {
        printk("[DEBUG] Connection failed with error, cleaning up\n");
        bt_conn_unref(default_conn);
        default_conn = NULL;
        return;
    }
    printk("[DEBUG] Connection successful, waiting for remote info...\n");
    k_sem_give(&sem_connected);
    printk("[DEBUG] Give up sem_connected.\n");

    struct bt_conn_le_phy_param phy_param = {
        .options    = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    int perr = bt_conn_le_phy_update(conn, &phy_param);
    if (perr) {
        printk("PHY update request failed (err %d)\n", perr);
    }else{
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
    printk("[DEBUG] Remote info available callback called\n");
    k_sem_give(&sem_connected);
    printk("[DEBUG] Gave sem_connected semaphore\n");
}

void le_param_updated(struct bt_conn *conn, uint16_t interval,
                      uint16_t latency, uint16_t timeout)
{
    printk("Connection parameters updated: interval %.2f ms, latency %d, timeout %d ms\n",
           (double)(interval * 1.25f), latency, timeout * 10);
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected               = connected_cb,
    .disconnected            = disconnected_cb,
    .remote_info_available   = remote_info_available_cb,
    .le_param_updated        = le_param_updated,
};

/* ---------- Scanner & name filter ---------- */
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
    char addr_str[BT_ADDR_LE_STR_LEN] = {0};
    char name[NAME_LEN] = {0};
    int err;

    if (default_conn) return;

    //if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
    //    return;
    //}

    bt_data_parse(ad, data_cb, name);
    if (strcmp(name, "PAwR sync sample")) return;

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    (void)bt_le_scan_stop();

    const struct bt_le_conn_param conn_param = {
        .interval_min = 0x18, /* 30ms */
        .interval_max = 0x28, /* 50ms */
        .latency      = 0,
        .timeout      = 400,  /* 4.0s */
    };

    printk("Creating connection to %s (%s) (rssi %d)\n", addr_str,
           addr->type == BT_ADDR_LE_PUBLIC ? "public" : "random", rssi);

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_param, &default_conn);
    if (err) {
        printk("Create conn to %s failed (%d)\n", addr_str, err);
    }
}

/* ---------- GATT for slot write ---------- */
static struct bt_uuid_128 pawr_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
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
    if (!attr->user_data) return BT_GATT_ITER_CONTINUE;

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
        net_buf_simple_init_with_data(&bufs[i], &backing_store[i], ARRAY_SIZE(backing_store[i]));
        net_buf_simple_reset(&bufs[i]);
        net_buf_simple_add_u8(&bufs[i], 3); /* length of manufacturer data */
        net_buf_simple_add_u8(&bufs[i], BT_DATA_MANUFACTURER_DATA);
        net_buf_simple_add_le16(&bufs[i], 0x0059); /* Nordic */
        printk("Buffer %d initialized with len %d\n", (int)i, bufs[i].len);
    }
}

/* ---------- Synchronizer slot config payload ---------- */
struct pawr_timing {
    uint8_t  subevent;
    uint8_t  response_slot;
    uint16_t total_devices;
    uint8_t  adv_sid;       /* extended advertising SID used for periodic */
} __packed;

static uint8_t num_synced;

/* ================================ MAIN ================================ */
int main(void)
{
    int err;
    struct bt_le_ext_adv *adv_pawr = NULL;  /* non-connectable PAwR set */
    struct bt_le_ext_adv *adv_conn = NULL;  /* optional connectable set */
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_write_params    write_params;
    struct pawr_timing             sync_config;

    init_bufs();

    printk("Starting PAwR Advertiser (two-phase)\n");
    printk("====================================\n");

    err = bt_enable(NULL);
    if (err) { 
        printk("Bluetooth init failed (%d)\n", err); 
        return 0; 
    }
    printk("Bluetooth initialized\n");

    reset_prov_sems();

    /* Create PAwR advertising set (EXT adv + periodic arm) */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &adv_pawr);
    if (err) { 
        printk("Failed to create adv (pawr) err %d\n", err); 
        return 0; 
    }

    /* If your SDK lacks bt_le_ext_adv_set_sid(), rely on scanned SID on the sync side */

    /* Start EXTENDED ADV now; lenient periodic will be started below */
    err = bt_le_ext_adv_start(adv_pawr, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) { 
        printk("Failed to start EXT adv (pawr) err %d\n", err); 
        return 0; 
    }
    printk("PAwR EXT adv is up\n");

    /* Lenient periodic arm ON during provisioning */
    struct bt_le_per_adv_param per_lenient;
    set_pawr_params_lenient(&per_lenient);
    g_active_num_slots    = 1;
    g_active_bitmap_bytes = 1;
    reset_bitmaps_for(g_active_num_slots);

	printk("[ADV] regular periodic interval = %u units (~%u ms) \n", per_adv_params.interval_max, per_adv_params.interval_max * 5/4);
    err = bt_le_per_adv_set_param(adv_pawr, &per_lenient);
    if (err) { 
        printk("Failed to set periodic (lenient) err %d\n", err); 
        return 0; 
    }
    err = bt_le_per_adv_start(adv_pawr);
    if (err) { 
        printk("Failed to start periodic (lenient) err %d\n", err); 
        return 0; 
    }
    printk("Periodic (lenient) up; PAST is valid\n");

    /* Optional connectable set (not required if we initiate) */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv_conn);
    if (!err) {
        err = bt_le_ext_adv_start(adv_conn, BT_LE_EXT_ADV_START_DEFAULT);
        if (err) {
            printk("ext adv start (conn) err %d\n", err);
        } else {
            printk("Connectable adv (provisioning) up\n");
        }
    }


    /* --------- Provisioning loop --------- */
    while (num_synced < CONFIG_BT_MAX_THROUGHPUT_DEVICES) {
        k_sleep(K_MSEC(5)); /* avoid BSIM churn */
        err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
        if (err) { 
            printk("scan start err %d\n", err); 
            break; 
        }
        printk("Scanning started\n");

        if (k_sem_take(&sem_connected, K_SECONDS(10))) {
            printk("Provisioning: no connection in time; retry\n");
            (void)bt_le_scan_stop();
            reset_prov_sems();
            continue;
        }
        (void)bt_le_scan_stop();
    

        printk("[PROV] provisioning (no PAST during this phase)\n");

        /* GATT: discover PAwR characteristic and write slot assignment */
        memset(&discover_params, 0, sizeof(discover_params));
        discover_params.uuid         = &pawr_char_uuid.uuid;
        discover_params.func         = discover_func;
        discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
        discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

        k_sem_reset(&sem_discovered);
        err = bt_gatt_discover(default_conn, &discover_params);
        if (err) {
            printk("[GATT] discover err %d or timeout\n", err);
            goto disconnect;
        }
        if(k_sem_take(&sem_discovered, K_SECONDS(10))){
            printk("[GATT] discover timed out \n");
            goto disconnect;
        }

        /* Write slot for this device */
        sync_config.subevent      = 0;
        sync_config.response_slot = num_synced;
        sync_config.total_devices = CONFIG_BT_MAX_THROUGHPUT_DEVICES;
        sync_config.adv_sid       = 0; /* BT_LE_EXT_ADV_NCONN uses SID 0 by default */

        memset(&write_params, 0, sizeof(write_params));
        write_params.func   = write_func;
        write_params.handle = pawr_attr_handle;
        write_params.offset = 0;
        write_params.data   = &sync_config;
        write_params.length = sizeof(sync_config);

        k_sem_reset(&sem_written);
        err = bt_gatt_write(default_conn, &write_params);
        if (err) {
            printk("[GATT] write err %d or timeout\n", err);
            goto disconnect;
        }
        if(k_sem_take(&sem_written, K_SECONDS(10))){
            printk("[GATT] write timed out \n");
            goto disconnect;
        }

        

        printk("PAwR slot %u written; disconnecting\n", (unsigned)num_synced);
        num_synced++;

    disconnect:
        /* grace before disconnect based on lenient interval */
        //k_sleep(K_MSEC(per_lenient.interval_max * 2));
        if (default_conn) {
            (void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            (void)k_sem_take(&sem_disconnected, K_MSEC(150));
            bt_conn_unref(default_conn);
            default_conn = NULL;
        }
        reset_prov_sems();
    }

    printk("Provisioning complete: %u devices\n", (unsigned)num_synced);

    /* --------- Switch to aggressive PAwR for data phase --------- */
    bt_le_per_adv_stop(adv_pawr);
    init_adv_params(); /* sets per_adv_params via set_pawr_params(..., NUM_RSP_SLOTS) */

    /* Align active slots with the configured periodic params to avoid -EINVAL */
    g_active_num_slots    = per_adv_params.num_response_slots;
    g_active_bitmap_bytes = BITMAP_BYTES_NEEDED(g_active_num_slots);
    reset_bitmaps_for(g_active_num_slots);

	printk("[ADV] aggressvive periodic interval = %u units (~%u ms) \n", per_adv_params.interval_max, per_adv_params.interval_max * 5/4);
    err = bt_le_per_adv_set_param(adv_pawr, &per_adv_params);
    if (err) { 
        printk("Failed to set periodic (aggr) err %d\n", err); 
        return 0; 
    }

    k_sleep(K_MSEC(per_adv_params.interval_max * 10)); // Longer delay to ensure stable advertising

    err = bt_le_per_adv_start(adv_pawr);
    k_sleep(K_MSEC(per_adv_params.interval_max * 3)); /* arm */
    if (err) { 
        printk("Failed to start periodic (aggr) err %d\n", err); 
        return 0; 
    }
    printk("Switched to aggressive PAwR (N=%u)\n", (unsigned)NUM_RSP_SLOTS);

    uint8_t repasted = 0;

	printk("[RE-PAST] Starting re-PAST pass for %u devices\n", (unsigned)CONFIG_BT_MAX_THROUGHPUT_DEVICES);

	/* Small settle to let controller start advertising robustly */
	k_sleep(K_MSEC(50));

	while (repasted < CONFIG_BT_MAX_THROUGHPUT_DEVICES) {
		k_sleep(K_MSEC(5)); /* avoid BSIM churn */

		int s_err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
		if (s_err) {
			printk("[RE-PAST] scan start err %d\n", s_err);
			break;
		}

		if (k_sem_take(&sem_connected, K_SECONDS(15))) {
			printk("[RE-PAST] no connection in time; retry\n");
			(void)bt_le_scan_stop();
			reset_prov_sems();
			continue;
		}
		(void)bt_le_scan_stop();

		/* Try a few times to hand over the new periodic */
		const int max_retries = 3;
		int try;
		int perr = -1;
		for (try = 0; try < max_retries; try++) {
			printk("[RE-PAST] Attempt %d to transfer periodic to peer\n", try + 1);
			perr = bt_le_per_adv_set_info_transfer(adv_pawr, default_conn, 0);
			if (perr == 0) {
				printk("[RE-PAST] transfer OK on attempt %d\n", try + 1);
				break;
			} else {
				printk("[RE-PAST] transfer err %d on attempt %d\n", perr, try + 1);
				// Longer backoff with exponential increase
                k_sleep(K_MSEC(100 * (1 << try)));
			}
		}

		if (perr != 0) {
			printk("[RE-PAST] transfer ultimately failed after %d attempts (err %d)\n", max_retries, perr);
		} else {
			repasted++;
			printk("[RE-PAST] OK (%u/%u)\n", repasted, CONFIG_BT_MAX_THROUGHPUT_DEVICES);
		}

        /* Keep connection alive to allow controller to send PAST HCI packet
        * and for peer to process events. Use a safer dwell.
        */
        k_sleep(K_MSEC(300));

		/* Now disconnect cleanly */
		if (default_conn != NULL) {
			(void)bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			(void)k_sem_take(&sem_disconnected, K_SECONDS(5));
			bt_conn_unref(default_conn);
			default_conn = NULL;
		}
		reset_prov_sems();
	}

	printk("All devices re-PASTed (successful: %u)\n", repasted);

    /* Re-assert periodic arm robustly: stop → set params → start.
     * Controller returns -EACCES if params are changed while running,
     * and -EALREADY if already in the desired state — both benign. */
    int serr = bt_le_per_adv_stop(adv_pawr);
    if (serr && serr != -EALREADY) {
        printk("Stop periodic (aggr) err %d\n", serr);
    }

    err = bt_le_per_adv_set_param(adv_pawr, &per_adv_params);
    if (err && err != -EACCES) {
        printk("Failed to set periodic (aggr) err %d\n", err);
        return 0;
    }

    err = bt_le_per_adv_start(adv_pawr);
    if (err && err != -EALREADY) {
        printk("Failed to start periodic (aggr) err %d\n", err);
        return 0;
    }

    
    /* --------- Data phase --------- */
    for (;;) {
        k_sleep(K_SECONDS(1));
        /* response_cb prints throughput periodically */
    }
    return 0;
}
