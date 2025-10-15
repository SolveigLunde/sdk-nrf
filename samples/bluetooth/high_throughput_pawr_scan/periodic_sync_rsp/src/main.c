/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>

#define MAX_INDIVIDUAL_RESPONSE_SIZE 247 // BLE spec limit for individual responses

static uint16_t actual_device_count = 0;
static uint16_t dynamic_rsp_size = 0;


// Calculate optimal response size based on actual device count from advertiser
static uint16_t calculate_optimal_response_size(uint16_t device_count) {
    if (device_count == 0) {
        device_count = 16; // Fallback only if not set
    }
    
    // Always use maximum PDU size
    uint16_t optimal_size = MAX_INDIVIDUAL_RESPONSE_SIZE - 3; // Hvis du går opp til 247 får jeg 3 byte overhead (som ga lav throughput). Enkel fiks var bare å ta -3. 
    printk("Using maximum PDU size: %u bytes\n", optimal_size);
    
    return optimal_size;
}

static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

static struct bt_le_per_adv_sync *default_sync;

/* Join/assignment state */
static uint8_t desired_subevent = 0; /* single subevent flow */
static int8_t assigned_slot = -1;    /* -1 means unassigned */
static uint32_t my_token = 0;        /* 0 means no token yet */
static uint8_t join_backoff_mod = 3; /* attempt every N events initially */
static uint8_t join_backoff_increase = 0; /* linear backoff growth */

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
    subevents[0] = desired_subevent;

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

    // Initialize dynamic response size if not set
    if (dynamic_rsp_size == 0) {
        dynamic_rsp_size = calculate_optimal_response_size(actual_device_count);
        if (actual_device_count > 0) {
            printk("Responder using dynamic response size: %d bytes\n", dynamic_rsp_size);
            printk("Configured for %d devices = %d bytes total\n", 
                   actual_device_count, actual_device_count * dynamic_rsp_size);
        } else {
            printk("Responder using fallback response size: %d bytes\n", dynamic_rsp_size);
            printk("Will be reconfigured when advertiser provides device count\n");
        }
    }

    if (buf && buf->len) {
        /* Parse control frame from advertiser (MSD):
         * [len][type=MSD][company_id(2)][ver(1)][flags(1)]
         * [open_len(1)][open_len bytes of bitmap]
         * [ack_count(1)][ack_count * {token(4), slot(1)}]
         * [rt_len(1)][retransmit_bitmap(rt_len)]
         */
        uint8_t open_len = 0;
        uint8_t open_bitmap[32]; /* supports up to 256 slots; grows if needed */
        memset(open_bitmap, 0, sizeof(open_bitmap));
        uint8_t ack_count = 0;
        const uint8_t *acks_ptr = NULL;
        uint8_t rt_len = 0;
        uint32_t retransmit_bitmap32 = 0;

        if (buf->len >= 9) {
            uint8_t *data = buf->data;
            if (data[1] == BT_DATA_MANUFACTURER_DATA && data[2] == 0x59 && data[3] == 0x00) {
                uint8_t idx = 4;
                if (buf->len > idx + 1) {
                    /* version */
                    idx += 1;
                }
                if (buf->len > idx + 1) {
                    /* flags */
                    idx += 1;
                }
                if (buf->len > idx) {
                    open_len = data[idx++];
                }
                if (open_len > sizeof(open_bitmap)) {
                    open_len = sizeof(open_bitmap);
                }
                if (buf->len >= idx + open_len) {
                    memcpy(open_bitmap, &data[idx], open_len);
                    idx += open_len;
                }
                if (buf->len > idx) {
                    ack_count = data[idx++];
                }
                if (buf->len >= idx + ack_count * 5) {
                    acks_ptr = &data[idx];
                    idx += ack_count * 5;
                }
                if (buf->len > idx) {
                    rt_len = data[idx++];
                }
                /* Extract up to 4 bytes of retransmit bitmap for quick checks */
                for (int i = 0; i < rt_len && i < 4; i++) {
                    if (buf->len > idx + i) {
                        retransmit_bitmap32 |= ((uint32_t)data[idx + i]) << (8 * i);
                    }
                }
            }
        }

        /* If we're unassigned, look for ACK with our token */
        if (assigned_slot < 0 && my_token != 0 && acks_ptr && ack_count > 0) {
            for (uint8_t a = 0; a < ack_count; a++) {
                uint32_t tok = sys_get_le32(&acks_ptr[a * 5]);
                uint8_t slot = acks_ptr[a * 5 + 4];
                if (tok == my_token) {
                    assigned_slot = slot;
                    printk("[JOIN] Acked: token 0x%08x assigned slot %d\n", my_token, assigned_slot);
                    break;
                }
            }
        }

        /* If still unassigned, probabilistically attempt a claim to reduce collisions */
        if (assigned_slot < 0) {
            uint32_t ev = info->periodic_event_counter;
            uint8_t mod = join_backoff_mod + join_backoff_increase;
            if (mod == 0) { mod = 1; }
            if ((ev % mod) == 0) {
                /* Choose a random open slot from variable-length bitmap */
                if (open_len > 0) {
                    uint16_t max_slots = (uint16_t)(open_len * 8);
                    uint16_t bitmap_slots_cap = (uint16_t)(sizeof(open_bitmap) * 8);
                    if (max_slots > bitmap_slots_cap) {
                        max_slots = bitmap_slots_cap;
                    }
                    uint16_t open_indices[256];
                    uint16_t open_count = 0;
                    for (uint16_t s = 0; s < max_slots; s++) {
                        uint8_t idx_b = s / 8;
                        uint8_t bit_b = s % 8;
                        if (open_bitmap[idx_b] & (1U << bit_b)) {
                            open_indices[open_count++] = s;
                        }
                    }
                    if (open_count > 0) {
                        uint32_t r = sys_rand32_get();
                        uint16_t pick = (uint16_t)(r % open_count);
                        uint8_t try_slot = (uint8_t)open_indices[pick];

                        if (my_token == 0) {
                            my_token = sys_rand32_get();
                            if (my_token == 0) {
                                my_token = 1; /* avoid 0 */
                            }
                        }

                        /* Build CLAIM payload: [0xC1][token(4B)] */
                        net_buf_simple_reset(&rsp_buf);
                        net_buf_simple_add_u8(&rsp_buf, 0xC1);
                        net_buf_simple_add_le32(&rsp_buf, my_token);

                        rsp_params.request_event = info->periodic_event_counter;
                        rsp_params.request_subevent = info->subevent;
                        rsp_params.response_subevent = info->subevent;
                        rsp_params.response_slot = try_slot;

                        int err2 = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);
                        if (err2) {
                            printk("[JOIN] Failed to send claim (slot %d, err %d)\n", try_slot, err2);
                        } else {
                            printk("[JOIN] Sent claim for slot %d with token 0x%08x\n", try_slot, my_token);
                        }

                        /* Linear backoff increase to spread retries if not acked */
                        if (join_backoff_increase < 10) {
                            join_backoff_increase++;
                        }
                    }
                }
            }
        }

        /* Generate test data only when assigned */
        if (assigned_slot >= 0) {
            /* Check retransmit bit for our slot */
            bool should_retransmit = false;
            if (assigned_slot < 32 && (retransmit_bitmap32 & (1U << assigned_slot))) {
                should_retransmit = true;
            }

            net_buf_simple_reset(&rsp_buf);
            for (int i = 0; i < dynamic_rsp_size; i++) {
                net_buf_simple_add_u8(&rsp_buf, (uint8_t)(assigned_slot + i));
            }

            rsp_params.request_event = info->periodic_event_counter;
            rsp_params.request_subevent = info->subevent;
            rsp_params.response_subevent = info->subevent;
            rsp_params.response_slot = assigned_slot;

            printk("Indication: subevent %d, responding in slot %d with %d bytes%s\n",
                   info->subevent, assigned_slot, rsp_buf.len,
                   should_retransmit ? " (retransmit)" : "");

            int err3 = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);
            if (err3) {
                printk("Failed to send response (err %d)\n", err3);
            }
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


/* Scan callback to detect periodic advertiser and create sync */
static void scan_recv_cb(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad)
{
    if (default_sync) {
        return;
    }

    if (!(info->adv_props & BT_GAP_ADV_PROP_EXT_ADV)) {
        return;
    }
    if (info->sid == 0xFF) {
        return;
    }

    struct bt_le_per_adv_sync_param param = {0};
    bt_addr_le_copy(&param.addr, info->addr);
    param.sid = info->sid;
    param.skip = 1;
    param.timeout = 1000; /* 10s */

    int err = bt_le_per_adv_sync_create(&param, &default_sync);
    if (!err) {
        printk("[SCAN] Creating periodic sync to SID %u\n", info->sid);
        (void)bt_le_scan_stop();
    }
}

static struct bt_le_scan_cb scan_cb = {
    .recv = scan_recv_cb,
};

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

    bt_le_scan_cb_register(&scan_cb);
    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, NULL);
    if (err) {
        printk("[SCAN] Error: start failed (err %d)\n", err);
        return 0;
    }

    printk("[SCAN] Waiting for periodic advertiser...\n");

    /* Wait for sync */
    err = k_sem_take(&sem_per_sync, K_FOREVER);
    if (err) {
        return 0;
    }

    while (true) {
        /* If sync is ever lost, restart scanning */
        (void)k_sem_take(&sem_per_sync_lost, K_FOREVER);
        assigned_slot = -1;
        my_token = 0;
        join_backoff_mod = 3;
        join_backoff_increase = 0;
        default_sync = NULL;
        (void)bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, NULL);
    }

    return 0;
}

