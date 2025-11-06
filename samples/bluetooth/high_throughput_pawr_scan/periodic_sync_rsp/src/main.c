#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "pawr_onboarding.h"

#define MAX_INDIVIDUAL_RESPONSE_SIZE 247
#define RESPONSE_PAYLOAD_SIZE (MAX_INDIVIDUAL_RESPONSE_SIZE - 3)

static K_SEM_DEFINE(sem_sync_established, 0, 1);
static K_SEM_DEFINE(sem_sync_lost, 0, 1);

static struct bt_conn *current_conn;
static struct bt_le_per_adv_sync *current_sync;

static struct pawr_slot_config assigned_slot_cfg;
static bool slot_assigned;

static struct bt_le_per_adv_response_params rsp_params;
NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, MAX_INDIVIDUAL_RESPONSE_SIZE + 4);

static uint16_t response_payload_size = RESPONSE_PAYLOAD_SIZE;

static const struct bt_le_adv_param adv_params = {
	.options = BT_LE_ADV_OPT_CONNECTABLE,
	.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
	.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	.peer = NULL,
};

static const struct bt_data adv_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		 sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void configure_sync_subevent(void)
{
	if (!current_sync || !slot_assigned) {
		return;
	}

	struct bt_le_per_adv_sync_subevent_params params = {
		.properties = 0,
		.num_subevents = 1,
	};
	uint8_t subevent = assigned_slot_cfg.subevent;
	params.subevents = &subevent;

	int err = bt_le_per_adv_sync_subevent(current_sync, &params);
	if (err) {
		printk("[SYNC] Failed to select subevent %u (err %d)\n", subevent, err);
	}
}

static ssize_t slot_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(struct pawr_slot_config)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	struct pawr_slot_config incoming;
	memcpy(&incoming, buf, len);
	incoming.payload_id = sys_le16_to_cpu(incoming.payload_id);

	assigned_slot_cfg = incoming;
	if (assigned_slot_cfg.response_slot >= PAWR_MAX_RESPONSE_SLOTS) {
		printk("[ONBOARD] Invalid slot %u received\n", assigned_slot_cfg.response_slot);
		return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
	}

	slot_assigned = true;
	configure_sync_subevent();

	printk("[ONBOARD] Accepted slot %u (subevent %u, total %u)\n",
		assigned_slot_cfg.response_slot,
		assigned_slot_cfg.subevent,
		assigned_slot_cfg.total_slots);

	return len;
}

static struct bt_gatt_attr pawr_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_PAWR_ONBOARDING_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_PAWR_SLOT_CHAR, BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, slot_write_cb, NULL),
};

static struct bt_gatt_service pawr_onboarding_svc = BT_GATT_SERVICE(pawr_attrs);

static void sync_established_cb(struct bt_le_per_adv_sync *sync,
				      struct bt_le_per_adv_sync_synced_info *info)
{
	current_sync = sync;
	configure_sync_subevent();
	k_sem_give(&sem_sync_established);

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
	printk("[SYNC] Synced via PAST to %s (%u subevents)\n", addr_str, info->num_subevents);
}

static void sync_terminated_cb(struct bt_le_per_adv_sync *sync,
				    const struct bt_le_per_adv_sync_term_info *info)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(info);
	current_sync = NULL;
	k_sem_give(&sem_sync_lost);
	printk("[SYNC] Lost periodic sync\n");
}

static void sync_recv_cb(struct bt_le_per_adv_sync *sync,
			       const struct bt_le_per_adv_sync_recv_info *info,
			       struct net_buf_simple *buf)
{
	ARG_UNUSED(sync);

	if (!slot_assigned) {
		return;
	}

	if (!buf || buf->len == 0U) {
		printk("[SYNC] Empty PAwR data\n");
		return;
	}

	net_buf_simple_reset(&rsp_buf);
	for (uint16_t i = 0; i < response_payload_size; i++) {
		net_buf_simple_add_u8(&rsp_buf,
			(uint8_t)(assigned_slot_cfg.response_slot + i));
	}

	rsp_params.request_event = info->periodic_event_counter;
	rsp_params.request_subevent = info->subevent;
	rsp_params.response_subevent = info->subevent;
	rsp_params.response_slot = assigned_slot_cfg.response_slot;

	int err = bt_le_per_adv_set_response_data(current_sync, &rsp_params, &rsp_buf);
	if (err) {
		printk("[SYNC] Failed to send response (err %d)\n", err);
	}
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_established_cb,
	.term = sync_terminated_cb,
	.recv = sync_recv_cb,
};

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("[CONN] Failed to connect (0x%02X)\n", err);
		return;
	}

	current_conn = bt_conn_ref(conn);

	struct bt_conn_le_phy_param phy_param = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
	};

	err = bt_conn_le_phy_update(conn, &phy_param);
	if (err) {
		printk("[CONN] PHY update request failed (%d)\n", err);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("[CONN] Disconnected (0x%02X)\n", reason);
	if (conn == current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

    /* Restart advertising so the proxy can reconnect if needed */
    int err = bt_le_adv_start(&adv_params, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (err && err != -EALREADY) {
        printk("[ADV] Failed to restart advertising (%d)\n", err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

static int start_advertising(void)
{
    int err = bt_le_adv_start(&adv_params, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
	if (err && err != -EALREADY) {
		printk("[ADV] Failed to start advertising (%d)\n", err);
		return err;
	}

	printk("[ADV] Advertising as onboarding peripheral\n");
	return 0;
}

int main(void)
{
	struct bt_le_per_adv_sync_transfer_param past_param = {
		.skip = 0,
		.timeout = 1000,
		.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE,
	};
	int err;

	printk("Starting PAwR synchronizer\n");

#if defined(CONFIG_BT_GATT_SERVICE_CHANGED)
	printk("[INIT] GATT server enabled\n");
#else
	printk("[INIT] GATT server not enabled!\n");
#endif

	err = bt_enable(NULL);
	if (err) {
		printk("[INIT] Bluetooth init failed (%d)\n", err);
		return err;
	}

	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	err = bt_gatt_service_register(&pawr_onboarding_svc);
	if (err) {
		printk("[INIT] Failed to register onboarding svc (%d)\n", err);
		return err;
	}
	const struct bt_gatt_attr *slot_attr = &pawr_onboarding_svc.attrs[1];
	uint16_t start_handle = bt_gatt_attr_get_handle(&pawr_onboarding_svc.attrs[0]);
	uint16_t end_handle = bt_gatt_attr_get_handle(&pawr_onboarding_svc.attrs[pawr_onboarding_svc.attr_count - 1]);
	printk("[INIT] onboarding svc registered, slot handle=0x%04x\n",
		bt_gatt_attr_get_handle(slot_attr));
	printk("[INIT] onboarding svc range 0x%04x-0x%04x\n", start_handle, end_handle);

	bt_gatt_service_changed(NULL, start_handle, end_handle);

	static uint8_t list_attrs(const struct bt_gatt_attr *attr, void *user_data)
	{
		char uuid_buf[BT_UUID_STR_LEN];
		const char *uuid_str = attr->uuid ? bt_uuid_to_str(attr->uuid, uuid_buf, sizeof(uuid_buf)) : "<none>";
		printk("[GATT] local attr handle=0x%04x uuid=%s perm=0x%02x\n",
			attr->handle, uuid_str, attr->perm);
		return BT_GATT_ITER_CONTINUE;
	}

	bt_gatt_foreach_attr(0x0001, 0xffff, list_attrs, NULL);

	err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
	if (err) {
		printk("[INIT] PAST subscribe failed (%d)\n", err);
		return err;
	}

	err = start_advertising();
	if (err && err != -EALREADY) {
		return err;
	}

	while (true) {
		if (k_sem_take(&sem_sync_established, K_FOREVER) != 0) {
			continue;
		}

		if (k_sem_take(&sem_sync_lost, K_FOREVER) != 0) {
			continue;
		}

		(void)start_advertising();
	}

	return 0;
}

