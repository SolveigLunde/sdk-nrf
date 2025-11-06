#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "onboarding_manager.h"
#include "pawr_onboarding.h"
#include "pawr_broadcaster.h"

#define TARGET_NAME "PAwR sync sample"
#define SCAN_RETRY_DELAY_MS 250
#ifdef CONFIG_BT_DEVICE_NAME_MAX
#define NAME_BUF_LEN CONFIG_BT_DEVICE_NAME_MAX
#else
#define NAME_BUF_LEN 30
#endif

struct slot_entry {
	bool valid;
	bool in_use;
	bt_addr_le_t addr;
};

static struct slot_entry slot_table[PAWR_MAX_RESPONSE_SLOTS];

static struct bt_conn *current_conn;
static bt_addr_le_t current_peer_addr;
static struct bt_le_ext_adv *adv_ref;

static uint8_t pending_slot = PAWR_MAX_RESPONSE_SLOTS;
static bool pending_slot_prev_valid;
static bool slot_committed;

static struct pawr_slot_config slot_config_buf;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_write_params write_params;

static struct k_work_delayable disconnect_work;
static struct k_work_delayable scan_work;

static bool scanning_enabled;

static void start_scan_if_possible(void);
static void cleanup_pending_slot(bool keep_assigned_record);
static void scan_recv_cb(const struct bt_le_scan_recv_info *info,
		struct net_buf_simple *ad);

static struct bt_le_scan_cb scan_cb = {
	.recv = scan_recv_cb,
};

static int find_slot_for_addr(const bt_addr_le_t *addr)
{
	for (int i = 0; i < PAWR_MAX_RESPONSE_SLOTS; i++) {
		if (slot_table[i].valid && (bt_addr_le_cmp(addr, &slot_table[i].addr) == 0)) {
			return i;
		}
	}
	return -1;
}

static int find_available_slot(void)
{
	for (int i = 0; i < PAWR_MAX_RESPONSE_SLOTS; i++) {
		if (!slot_table[i].valid) {
			return i;
		}
	}
	for (int i = 0; i < PAWR_MAX_RESPONSE_SLOTS; i++) {
		if (!slot_table[i].in_use) {
			return i;
		}
	}
	return -1;
}

static void stop_scan(void)
{
	if (!scanning_enabled) {
		return;
	}

	int err = bt_le_scan_stop();
	if (err && err != -EALREADY) {
		printk("[SCAN] stop failed (err %d)\n", err);
	}
	scanning_enabled = false;
}

static void scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	start_scan_if_possible();
}

static bool name_match_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_BUF_LEN - 1U);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static bool ad_matches_target(struct net_buf_simple *ad)
{
	char name[NAME_BUF_LEN];

	memset(name, 0, sizeof(name));
	bt_data_parse(ad, name_match_cb, name);

	return (strlen(name) > 0U) && (strcmp(name, TARGET_NAME) == 0);
}

static void scan_recv_cb(const struct bt_le_scan_recv_info *info,
		   struct net_buf_simple *ad)
{
	if (current_conn) {
		return;
	}

	if (!(info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE)) {
		return;
	}

	if (!ad_matches_target(ad)) {
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
	printk("[SCAN] found %s (SID %u)\n", addr_str, info->sid);

	stop_scan();

	int err = bt_conn_le_create(info->addr, BT_CONN_LE_CREATE_CONN,
					BT_LE_CONN_PARAM_DEFAULT, &current_conn);
	if (err) {
		printk("[SCAN] create conn failed (%d)\n", err);
		bt_addr_le_copy(&current_peer_addr, BT_ADDR_LE_ANY);
		k_work_reschedule(&scan_work, K_MSEC(SCAN_RETRY_DELAY_MS));
		return;
	}

	bt_addr_le_copy(&current_peer_addr, info->addr);
}

static void start_scan_if_possible(void)
{
	if (scanning_enabled || current_conn) {
		return;
	}

	int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, NULL);
	if (err) {
		if (err != -EALREADY) {
			printk("[SCAN] start failed (%d)\n", err);
			k_work_reschedule(&scan_work, K_MSEC(SCAN_RETRY_DELAY_MS));
		}
		return;
	}

	scanning_enabled = true;
	printk("[SCAN] scanning for synchronizers\n");
}

static void disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (current_conn) {
		int err = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (err && err != -ENOTCONN) {
			printk("[CONN] disconnect failed (err %d)\n", err);
		}
	}
}

static int begin_slot_assignment(struct bt_conn *conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	int slot = find_slot_for_addr(addr);
	bool new_mapping = false;

	if (slot < 0) {
		slot = find_available_slot();
		new_mapping = true;
	}

	if (slot < 0) {
		printk("[ONBOARD] No slots available for new device\n");
		return -ENOBUFS;
	}

	pending_slot = slot;
	pending_slot_prev_valid = slot_table[slot].valid;
	slot_committed = false;

	if (new_mapping || !slot_table[slot].valid) {
		bt_addr_le_copy(&slot_table[slot].addr, addr);
		slot_table[slot].valid = true;
	}
	slot_table[slot].in_use = false;

	slot_config_buf.subevent = PAWR_DEFAULT_SUBEVENT;
	slot_config_buf.response_slot = slot;
	slot_config_buf.payload_id = sys_cpu_to_le16(slot);
	slot_config_buf.total_slots = pawr_broadcaster_total_slots();

	int err = bt_le_per_adv_set_info_transfer(adv_ref, conn, slot);
	if (err) {
		printk("[ONBOARD] PAST send failed (%d)\n", err);
		return err;
	}

	discover_params.uuid = BT_UUID_PAWR_SLOT_CHAR;
	discover_params.func = NULL; /* will set below */
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	return 0;
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params);
static void write_cb(struct bt_conn *conn, uint8_t err,
			   struct bt_gatt_write_params *params);

static int start_discovery(struct bt_conn *conn)
{
	discover_params.uuid = BT_UUID_PAWR_SLOT_CHAR;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	int err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("[ONBOARD] discovery failed (%d)\n", err);
	}
	return err;
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(params);

	if (!attr) {
		printk("[ONBOARD] Slot characteristic not found\n");
		cleanup_pending_slot(false);
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
		return BT_GATT_ITER_STOP;
	}

	printk("[GATT] Discover attr handle=0x%04x uuid=%s type=0x%04x len=%u\n",
		attr->handle,
		bt_uuid_str(attr->uuid),
		attr->uuid ? attr->uuid->type : 0,
		attr->read ? attr->read(conn, attr, NULL, 0, 0) : 0);

	struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
	if (bt_uuid_cmp(chrc->uuid, BT_UUID_PAWR_SLOT_CHAR) != 0) {
		return BT_GATT_ITER_CONTINUE;
	}

	write_params.func = write_cb;
	write_params.handle = chrc->value_handle;
	write_params.offset = 0U;
	write_params.data = &slot_config_buf;
	write_params.length = sizeof(slot_config_buf);

	int err = bt_gatt_write(conn, &write_params);
	if (err) {
		printk("[ONBOARD] write start failed (%d)\n", err);
		cleanup_pending_slot(false);
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
	}

	return BT_GATT_ITER_STOP;
}

static void write_cb(struct bt_conn *conn, uint8_t err,
			   struct bt_gatt_write_params *params)
{
	ARG_UNUSED(params);

	if (err) {
		printk("[ONBOARD] slot write failed (err %u)\n", err);
		cleanup_pending_slot(false);
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
		return;
	}

	if (pending_slot >= PAWR_MAX_RESPONSE_SLOTS) {
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
		return;
	}

	slot_table[pending_slot].in_use = true;
	slot_committed = true;
	pawr_broadcaster_set_slot_expected(pending_slot, true);

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(&current_peer_addr, addr_str, sizeof(addr_str));
	printk("[ONBOARD] Assigned slot %u to %s\n", pending_slot, addr_str);

	k_work_reschedule(&disconnect_work, K_MSEC(30));
}

static void cleanup_pending_slot(bool keep_assigned_record)
{
	if (pending_slot >= PAWR_MAX_RESPONSE_SLOTS) {
		return;
	}

	if (!keep_assigned_record) {
		pawr_broadcaster_set_slot_expected(pending_slot, false);
		slot_table[pending_slot].in_use = false;
		if (!pending_slot_prev_valid) {
			slot_table[pending_slot].valid = false;
			bt_addr_le_copy(&slot_table[pending_slot].addr, BT_ADDR_LE_ANY);
		}
	}

	pending_slot = PAWR_MAX_RESPONSE_SLOTS;
	pending_slot_prev_valid = false;
	slot_committed = false;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

	if (err) {
		printk("[CONN] Failed to connect to %s (0x%02X)\n", addr_str, err);
		if (conn == current_conn) {
			bt_conn_unref(current_conn);
			current_conn = NULL;
			k_work_reschedule(&scan_work, K_MSEC(SCAN_RETRY_DELAY_MS));
		}
		return;
	}

	printk("[CONN] Connected to %s\n", addr_str);

	struct bt_conn_le_phy_param phy_param = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
	};

	int err_phy = bt_conn_le_phy_update(conn, &phy_param);
	if (err_phy) {
		printk("[CONN] PHY update request failed (%d)\n", err_phy);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	printk("[CONN] Disconnected from %s (0x%02X)\n", addr_str, reason);

	if (conn == current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	k_work_cancel_delayable(&disconnect_work);

	cleanup_pending_slot(slot_committed);

	k_work_reschedule(&scan_work, K_MSEC(50));
}

static void remote_info_available_cb(struct bt_conn *conn,
				   struct bt_conn_remote_info *info)
{
	ARG_UNUSED(info);

	if (conn != current_conn) {
		return;
	}

	int err = begin_slot_assignment(conn);
	if (err) {
		cleanup_pending_slot(false);
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
		return;
	}

	err = start_discovery(conn);
	if (err) {
		cleanup_pending_slot(false);
		k_work_reschedule(&disconnect_work, K_NO_WAIT);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.remote_info_available = remote_info_available_cb,
};

int onboarding_manager_init(struct bt_le_ext_adv *pawr_adv)
{
	adv_ref = pawr_adv;
	k_work_init_delayable(&disconnect_work, disconnect_work_handler);
	k_work_init_delayable(&scan_work, scan_work_handler);
	bt_le_scan_cb_register(&scan_cb);
	bt_conn_cb_register(&conn_callbacks);
	memset(slot_table, 0, sizeof(slot_table));
	return 0;
}

void onboarding_manager_start(void)
{
	start_scan_if_possible();
}

void onboarding_manager_handle_slot_idle(uint8_t slot)
{
	if (slot >= PAWR_MAX_RESPONSE_SLOTS) {
		return;
	}

	if (!slot_table[slot].valid) {
		return;
	}

	printk("[ONBOARD] Slot %u idle, clearing assignment\n", slot);
	pawr_broadcaster_set_slot_expected(slot, false);
	slot_table[slot].in_use = false;
	slot_table[slot].valid = false;
	bt_addr_le_copy(&slot_table[slot].addr, BT_ADDR_LE_ANY);
	k_work_reschedule(&scan_work, K_MSEC(10));
}

