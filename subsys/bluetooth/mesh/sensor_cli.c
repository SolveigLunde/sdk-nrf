/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>
#include <bluetooth/mesh/sensor_cli.h>
#include "model_utils.h"
#include "sensor.h"
#include "mesh/net.h"

#define LOG_LEVEL CONFIG_BT_MESH_MODEL_LOG_LEVEL
#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(bt_mesh_sensor_cli);

struct list_rsp {
	struct bt_mesh_sensor_info *sensors;
	uint32_t count;
};

struct settings_rsp {
	uint16_t id;
	uint16_t *ids;
	uint32_t count;
};

struct setting_rsp {
	uint16_t id;
	uint16_t setting_id;
	struct bt_mesh_sensor_setting_status *setting;
};

struct sensor_data_list_rsp {
	struct bt_mesh_sensor_data *sensors;
	uint32_t count;
};

struct series_data_rsp {
	struct bt_mesh_sensor_series_entry *entries;
	const struct bt_mesh_sensor_value *col_start;
	uint16_t id;
	uint32_t count;
};

struct cadence_rsp {
	uint16_t id;
	struct bt_mesh_sensor_cadence_status *cadence;
};


static void unknown_type(struct bt_mesh_sensor_cli *cli,
			 struct bt_mesh_msg_ctx *ctx, uint16_t id, uint32_t op)
{
	if (cli->cb && cli->cb->unknown_type) {
		cli->cb->unknown_type(cli, ctx, id, op);
	}
}

static int handle_descriptor_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				    struct net_buf_simple *buf)
{
	if (buf->len != 2 && buf->len % 8) {
		return -EMSGSIZE;
	}

	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct list_rsp *ack_ctx = NULL;
	uint32_t count = 0;

	(void)bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_DESCRIPTOR_STATUS,
					ctx->addr, (void **)&ack_ctx);

	/* A packet with only the sensor ID means that the given sensor doesn't
	 * exist on the sensor server.
	 */
	if (buf->len == 2) {
		goto yield_ack;
	}

	struct bt_mesh_sensor_info sensor;

	while (buf->len >= 8) {

		sensor_descriptor_decode(buf, &sensor);

		if (cli->cb && cli->cb->sensor) {
			cli->cb->sensor(cli, ctx, &sensor);
		}

		if (ack_ctx && count < ack_ctx->count) {
			ack_ctx->sensors[count++] = sensor;
		}
	}

yield_ack:
	if (ack_ctx) {
		ack_ctx->count = count;
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int handle_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct sensor_data_list_rsp *rsp = NULL;
	uint32_t count = 0;
	int err;

	(void)bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_STATUS, ctx->addr,
					(void **)&rsp);

	while (buf->len) {
		const struct bt_mesh_sensor_type *type;
		uint8_t length;
		uint16_t id;

		sensor_status_id_decode(buf, &length, &id);
		if (length == 0) {
			if (rsp && rsp->count == 1 && rsp->sensors[0].type &&
			    rsp->sensors[0].type->id == id) {
				rsp->sensors[0].type = NULL;
				bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
				rsp = NULL;
			}

			continue;
		}

		type = bt_mesh_sensor_type_get(id);
		if (!type) {
			if (buf->len < length) {
				LOG_WRN("Invalid length for 0x%04x: %u", id,
					length);
				return -EMSGSIZE;
			}
			net_buf_simple_pull(buf, length);
			unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_STATUS);
			continue;
		}

		uint8_t expected_len = sensor_value_len(type);

		if (length != expected_len) {
			LOG_WRN("Invalid length for 0x%04x: %u (expected %u)",
				id, length, expected_len);
			return -EMSGSIZE;
		}

		struct bt_mesh_sensor_value value[CONFIG_BT_MESH_SENSOR_CHANNELS_MAX];

		err = sensor_value_decode(buf, type, value);
		if (err) {
			LOG_ERR("Invalid format, err=%d", err);
			return err; /* Invalid format, should ignore message */
		}

		if (cli->cb && cli->cb->data) {
			cli->cb->data(cli, ctx, type, value);
		}

		if (rsp && count <= rsp->count) {
			memcpy(rsp->sensors[count].value, value,
			       sizeof(struct bt_mesh_sensor_value) * type->channel_count);

			rsp->sensors[count].type = type;
			++count;
		}
	}

	if (rsp) {
		rsp->count = count;
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int parse_series_entry(const struct bt_mesh_sensor_type *type,
			      const struct bt_mesh_sensor_format *col_format,
			      struct net_buf_simple *buf,
			      struct bt_mesh_sensor_series_entry *entry)
{
	if (col_format == NULL) {
		// Indexed column, decode only value
		memset(entry, 0, sizeof(struct bt_mesh_sensor_series_entry));
		return sensor_value_decode(buf, type, entry->value);
	}

	return sensor_column_decode(buf, type, &entry->column, entry->value);
}

static int handle_column_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct series_data_rsp *rsp;
	const struct bt_mesh_sensor_format *col_format;
	const struct bt_mesh_sensor_type *type;
	int err;

	uint16_t id = net_buf_simple_pull_le16(buf);

	type = bt_mesh_sensor_type_get(id);
	if (!type) {
		unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_COLUMN_STATUS);
		return -ENOENT;
	}

	struct bt_mesh_sensor_series_entry entry;

	col_format = bt_mesh_sensor_column_format_get(type);

	err = parse_series_entry(type, col_format, buf, &entry);
	if (err == -ENOENT) {
		/* The entry doesn't exist */
		goto yield_ack;
	}

	if (err) {
		return err; /* Invalid format, should ignore message */
	}

	if (cli->cb && cli->cb->series_entry) {
		cli->cb->series_entry(cli, ctx, type, 0, 1, &entry);
	}

yield_ack:
	if (bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_COLUMN_STATUS, ctx->addr,
								  (void **)&rsp)) {
		/* If column format exists verify Raw Value A */
		if (col_format &&
			col_format->cb->compare(rsp->col_start, &entry.column.start) != 0) {
			return 0;
		}

		if (err) {
			rsp->count = 0;
		} else {
			rsp->entries[0] = entry;
			rsp->count = 1;
		}
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int handle_series_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	const struct bt_mesh_sensor_format *col_format;
	const struct bt_mesh_sensor_type *type;
	struct series_data_rsp *rsp = NULL;

	uint16_t id = net_buf_simple_pull_le16(buf);

	type = bt_mesh_sensor_type_get(id);
	if (!type) {
		unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_SERIES_STATUS);
		return -ENOENT;
	}

	if (bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_SERIES_STATUS, ctx->addr,
				      (void **)&rsp)) {
		if (rsp->id != id) {
			rsp = NULL;
		}
	}

	size_t val_len = sensor_value_len(type);

	col_format = bt_mesh_sensor_column_format_get(type);
	if (col_format != NULL) {
		val_len += (col_format->size * 2);
	}

	if (!val_len) {
		return -ENOTSUP;
	}

	uint8_t count = buf->len / val_len;

	for (uint8_t i = 0; i < count; i++) {
		struct bt_mesh_sensor_series_entry entry;
		int err;

		err = parse_series_entry(type, col_format, buf, &entry);
		if (err) {
			LOG_ERR("Failed parsing column %u (err: %d)", i, err);
			return err;
		}

		if (cli->cb && cli->cb->series_entry) {
			cli->cb->series_entry(cli, ctx, type, i, count, &entry);
		}

		if (rsp && i < rsp->count) {
			rsp->entries[i] = entry;
		}
	}

	if (rsp) {
		rsp->count = count;
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int handle_cadence_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				 struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct cadence_rsp *rsp;
	int err;

	uint16_t id = net_buf_simple_pull_le16(buf);
	const struct bt_mesh_sensor_type *type = bt_mesh_sensor_type_get(id);

	if (!type) {
		unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_CADENCE_STATUS);
		return -ENOENT;
	}

	if (buf->len == 0 || type->channel_count != 1) {
		LOG_WRN("Type 0x%04x doesn't support cadence", id);
		err = -ENOTSUP;
		goto yield_ack;
	}

	struct bt_mesh_sensor_cadence_status cadence;

	err = sensor_cadence_decode(buf, type, &cadence.fast_period_div,
				    &cadence.min_int, &cadence.threshold);
	if (err) {
		return err;
	}

	if (cli->cb && cli->cb->cadence) {
		cli->cb->cadence(cli, ctx, type, &cadence);
	}

yield_ack:
	if (bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_CADENCE_STATUS, ctx->addr,
				      (void **)&rsp) &&
	    rsp->id == id) {
		if (err) {
			rsp->cadence = NULL;
		} else {
			*rsp->cadence = cadence;
		}

		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int handle_settings_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct settings_rsp *rsp;

	if (buf->len % 2) {
		return -EMSGSIZE;
	}

	uint16_t id = net_buf_simple_pull_le16(buf);
	const struct bt_mesh_sensor_type *type = bt_mesh_sensor_type_get(id);

	if (!type) {
		unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_SETTINGS_STATUS);
		return -ENOENT;
	}

	/* The list may be unaligned: */
	uint16_t ids[(BT_MESH_RX_SDU_MAX -
		   BT_MESH_MODEL_OP_LEN(BT_MESH_SENSOR_OP_SETTINGS_STATUS) -
		   2) /
		  sizeof(uint16_t)];
	uint32_t count = buf->len / 2;

	memcpy(ids, net_buf_simple_pull_mem(buf, buf->len),
	       count * sizeof(uint16_t));

	if (cli->cb && cli->cb->settings) {
		cli->cb->settings(cli, ctx, type, ids, count);
	}

	if (bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_SETTINGS_STATUS, ctx->addr,
				      (void **)&rsp) &&
	    rsp->id == id) {
		memcpy(rsp->ids, ids, sizeof(uint16_t) * MIN(count, rsp->count));
		rsp->count = count;
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

static int handle_setting_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				 struct net_buf_simple *buf)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;
	struct setting_rsp *rsp;
	int err;

	uint16_t id = net_buf_simple_pull_le16(buf);
	uint16_t setting_id = net_buf_simple_pull_le16(buf);
	struct bt_mesh_sensor_setting_status setting = { 0 };
	uint8_t access;

	if (buf->len == 0) {
		goto yield_ack;
	}

	access = net_buf_simple_pull_u8(buf);
	if (access != 0x01 && access != 0x03) {
		return -EINVAL;
	}

	const struct bt_mesh_sensor_type *type = bt_mesh_sensor_type_get(id);

	if (!type) {
		unknown_type(cli, ctx, id, BT_MESH_SENSOR_OP_SETTING_STATUS);
		return -ENOENT;
	}


	setting.type = bt_mesh_sensor_type_get(setting_id);
	if (!setting.type) {
		unknown_type(
			cli, ctx, setting_id, BT_MESH_SENSOR_OP_SETTING_STATUS);
		return -ENOENT;
	}

	setting.writable = (access == 0x03);

	/* If we attempted setting an unwritable value, the server omits the
	 * setting value. This is a legal response, but it should make the set()
	 * function return -EACCES.
	 */
	if (buf->len == 0 && !setting.writable) {
		goto yield_ack;
	}

	err = sensor_value_decode(buf, setting.type, setting.value);
	if (err) {
		return err;
	}

	if (cli->cb && cli->cb->setting_status) {
		cli->cb->setting_status(cli, ctx, type, &setting);
	}

yield_ack:
	if (bt_mesh_msg_ack_ctx_match(&cli->ack_ctx, BT_MESH_SENSOR_OP_SETTING_STATUS, ctx->addr,
				      (void **)&rsp) &&
	    rsp->id == id && rsp->setting_id == setting_id) {
		*rsp->setting = setting;
		bt_mesh_msg_ack_ctx_rx(&cli->ack_ctx);
	}

	return 0;
}

const struct bt_mesh_model_op _bt_mesh_sensor_cli_op[] = {
	{
		BT_MESH_SENSOR_OP_DESCRIPTOR_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_DESCRIPTOR_STATUS),
		handle_descriptor_status,
	},
	{
		BT_MESH_SENSOR_OP_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_STATUS),
		handle_status,
	},
	{
		BT_MESH_SENSOR_OP_COLUMN_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_COLUMN_STATUS),
		handle_column_status,
	},
	{
		BT_MESH_SENSOR_OP_SERIES_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_SERIES_STATUS),
		handle_series_status,
	},
	{
		BT_MESH_SENSOR_OP_CADENCE_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_CADENCE_STATUS),
		handle_cadence_status,
	},
	{
		BT_MESH_SENSOR_OP_SETTINGS_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_SETTINGS_STATUS),
		handle_settings_status,
	},
	{
		BT_MESH_SENSOR_OP_SETTING_STATUS,
		BT_MESH_LEN_MIN(BT_MESH_SENSOR_MSG_MINLEN_SETTING_STATUS),
		handle_setting_status,
	},
	BT_MESH_MODEL_OP_END,
};

static int sensor_cli_init(const struct bt_mesh_model *model)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;

	cli->model = model;
	cli->pub.msg = &cli->pub_buf;
	net_buf_simple_init_with_data(&cli->pub_buf, cli->pub_data,
				      sizeof(cli->pub_data));
	bt_mesh_msg_ack_ctx_init(&cli->ack_ctx);

	return 0;
}

static void sensor_cli_reset(const struct bt_mesh_model *model)
{
	struct bt_mesh_sensor_cli *cli = model->rt->user_data;

	net_buf_simple_reset(cli->pub.msg);
	bt_mesh_msg_ack_ctx_reset(&cli->ack_ctx);
}

const struct bt_mesh_model_cb _bt_mesh_sensor_cli_cb = {
	.init = sensor_cli_init,
	.reset = sensor_cli_reset,
};

int bt_mesh_sensor_cli_desc_all_get(struct bt_mesh_sensor_cli *cli,
				    struct bt_mesh_msg_ctx *ctx,
				    struct bt_mesh_sensor_info *sensors,
				    uint32_t *count)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_DESCRIPTOR_GET,
				 BT_MESH_SENSOR_MSG_MINLEN_DESCRIPTOR_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_DESCRIPTOR_GET);

	struct list_rsp list_rsp = {
		.sensors = sensors,
		.count = count ? *count : 0,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_DESCRIPTOR_STATUS,
		.user_data = &list_rsp,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, sensors ? &rsp_ctx : NULL);
	if (count && !err) {
		*count = list_rsp.count;
	}

	return err;
}

int bt_mesh_sensor_cli_desc_get(struct bt_mesh_sensor_cli *cli,
				struct bt_mesh_msg_ctx *ctx,
				const struct bt_mesh_sensor_type *sensor,
				struct bt_mesh_sensor_descriptor *rsp)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_DESCRIPTOR_GET,
				 BT_MESH_SENSOR_MSG_MAXLEN_DESCRIPTOR_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_DESCRIPTOR_GET);

	net_buf_simple_add_le16(&msg, sensor->id);

	struct bt_mesh_sensor_info info = {
		sensor->id,
	};
	struct list_rsp list_rsp = {
		.sensors = &info,
		.count = 1,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_DESCRIPTOR_STATUS,
		.user_data = &list_rsp,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	if (list_rsp.count == 0) {
		return -ENOTSUP;
	}

	*rsp = info.descriptor;

	return 0;
}

int bt_mesh_sensor_cli_cadence_get(struct bt_mesh_sensor_cli *cli,
				   struct bt_mesh_msg_ctx *ctx,
				   const struct bt_mesh_sensor_type *sensor,
				   struct bt_mesh_sensor_cadence_status *rsp)
{
	if (sensor->channel_count != 1) {
		return -ENOTSUP;
	}

	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_CADENCE_GET, 2);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_CADENCE_GET);

	net_buf_simple_add_le16(&msg, sensor->id);

	struct cadence_rsp rsp_data = {
		.id = sensor->id,
		.cadence = rsp,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_CADENCE_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	/* Status handler clears the cadence member if server indicates that
	 * cadence isn't supported.
	 */
	if (!rsp_data.cadence) {
		return -ENOTSUP;
	}

	return 0;
}

int bt_mesh_sensor_cli_cadence_set(
	struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
	const struct bt_mesh_sensor_type *sensor,
	const struct bt_mesh_sensor_cadence_status *cadence,
	struct bt_mesh_sensor_cadence_status *rsp)
{
	if (sensor->channel_count != 1) {
		return -ENOTSUP;
	}

	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_CADENCE_SET,
				 BT_MESH_SENSOR_MSG_MAXLEN_CADENCE_SET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_CADENCE_SET);

	net_buf_simple_add_le16(&msg, sensor->id);

	struct cadence_rsp rsp_data = {
		.id = sensor->id,
		.cadence = rsp,
	};

	err = sensor_cadence_encode(&msg, sensor, cadence->fast_period_div,
				    cadence->min_int, &cadence->threshold);
	if (err) {
		return err;
	}

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_CADENCE_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	/* Status handler clears the cadence member if server indicates that
	 * cadence isn't supported.
	 */
	if (!rsp_data.cadence) {
		return -ENOTSUP;
	}

	return 0;
}

int bt_mesh_sensor_cli_cadence_set_unack(
	struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
	const struct bt_mesh_sensor_type *sensor,
	const struct bt_mesh_sensor_cadence_status *cadence)
{
	if (sensor->channel_count != 1) {
		return -ENOTSUP;
	}

	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg,
				 BT_MESH_SENSOR_OP_CADENCE_SET_UNACKNOWLEDGED,
				 BT_MESH_SENSOR_MSG_MAXLEN_CADENCE_SET);
	bt_mesh_model_msg_init(&msg,
			       BT_MESH_SENSOR_OP_CADENCE_SET_UNACKNOWLEDGED);

	net_buf_simple_add_le16(&msg, sensor->id);

	err = sensor_cadence_encode(&msg, sensor, cadence->fast_period_div,
				    cadence->min_int, &cadence->threshold);
	if (err) {
		return err;
	}

	return bt_mesh_msg_send(cli->model, ctx, &msg);
}

int bt_mesh_sensor_cli_settings_get(struct bt_mesh_sensor_cli *cli,
				    struct bt_mesh_msg_ctx *ctx,
				    const struct bt_mesh_sensor_type *sensor,
				    uint16_t *ids, uint32_t *count)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_SETTINGS_GET,
				 BT_MESH_SENSOR_MSG_LEN_SETTINGS_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_SETTINGS_GET);

	net_buf_simple_add_le16(&msg, sensor->id);

	struct settings_rsp rsp = {
		.id = sensor->id,
		.ids = ids,
		.count = count ? *count : 0,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_SETTINGS_STATUS,
		.user_data = &rsp,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, ids ? &rsp_ctx : NULL);
	if (!count || err) {
		return err;
	}

	*count = rsp.count;

	return err;
}

int bt_mesh_sensor_cli_setting_get(struct bt_mesh_sensor_cli *cli,
				   struct bt_mesh_msg_ctx *ctx,
				   const struct bt_mesh_sensor_type *sensor,
				   const struct bt_mesh_sensor_type *setting,
				   struct bt_mesh_sensor_setting_status *rsp)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_SETTING_GET,
				 BT_MESH_SENSOR_MSG_LEN_SETTING_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_SETTING_GET);

	net_buf_simple_add_le16(&msg, sensor->id);
	net_buf_simple_add_le16(&msg, setting->id);

	struct setting_rsp rsp_data = {
		.id = sensor->id,
		.setting_id = setting->id,
		.setting = rsp,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_SETTING_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	if (!rsp->type) {
		return -ENOENT;
	}

	return 0;
}

int bt_mesh_sensor_cli_setting_set(struct bt_mesh_sensor_cli *cli,
				   struct bt_mesh_msg_ctx *ctx,
				   const struct bt_mesh_sensor_type *sensor,
				   const struct bt_mesh_sensor_type *setting,
				   const struct bt_mesh_sensor_value *value,
				   struct bt_mesh_sensor_setting_status *rsp)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_SETTING_SET,
				 BT_MESH_SENSOR_MSG_MAXLEN_SETTING_SET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_SETTING_SET);

	net_buf_simple_add_le16(&msg, sensor->id);
	net_buf_simple_add_le16(&msg, setting->id);
	err = sensor_value_encode(&msg, setting, value);
	if (err) {
		return err;
	}

	struct setting_rsp rsp_data = {
		.id = sensor->id,
		.setting_id = setting->id,
		.setting = rsp,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_SETTING_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	if (!rsp->type) {
		return -ENOENT;
	}

	if (!rsp->writable) {
		return -EACCES;
	}

	return 0;
}

int bt_mesh_sensor_cli_setting_set_unack(
	struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
	const struct bt_mesh_sensor_type *sensor,
	const struct bt_mesh_sensor_type *setting,
	const struct bt_mesh_sensor_value *value)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_SETTING_SET_UNACKNOWLEDGED,
				 BT_MESH_SENSOR_MSG_MAXLEN_SETTING_SET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_SETTING_SET_UNACKNOWLEDGED);

	net_buf_simple_add_le16(&msg, sensor->id);
	net_buf_simple_add_le16(&msg, setting->id);
	err = sensor_value_encode(&msg, setting, value);
	if (err) {
		return err;
	}

	return bt_mesh_msg_send(cli->model, ctx, &msg);
}

int bt_mesh_sensor_cli_all_get(struct bt_mesh_sensor_cli *cli,
			       struct bt_mesh_msg_ctx *ctx,
			       struct bt_mesh_sensor_data *sensors,
			       uint32_t *count)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_GET,
				 BT_MESH_SENSOR_MSG_MINLEN_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_GET);

	struct sensor_data_list_rsp rsp_data = { .count = count ? *count : 0,
						 .sensors = sensors };

	if (sensors && count) {
		memset(sensors, 0, sizeof(*sensors) * (*count));
	}

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, sensors ? &rsp_ctx : NULL);
	if (!count || err) {
		return err;
	}

	if (rsp_data.count == 0) {
		return -ENODEV;
	}

	*count = rsp_data.count;

	return 0;
}

int bt_mesh_sensor_cli_get(struct bt_mesh_sensor_cli *cli,
			   struct bt_mesh_msg_ctx *ctx,
			   const struct bt_mesh_sensor_type *sensor,
			   struct bt_mesh_sensor_value *rsp)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_GET,
				 BT_MESH_SENSOR_MSG_MAXLEN_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_GET);
	net_buf_simple_add_le16(&msg, sensor->id);

	struct bt_mesh_sensor_data sensor_data = {
		.type = sensor,
	};
	struct sensor_data_list_rsp rsp_data = { .count = 1,
						 .sensors = &sensor_data };

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	if (!sensor_data.type) {
		return -ENODEV;
	}

	memcpy(rsp, sensor_data.value, sizeof(struct bt_mesh_sensor_value) * sensor->channel_count);

	return 0;
}

int bt_mesh_sensor_cli_series_entry_get(
	struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
	const struct bt_mesh_sensor_type *sensor,
	const union bt_mesh_sensor_column_key *column,
	struct bt_mesh_sensor_series_entry *rsp)
{
	const struct bt_mesh_sensor_format *col_format;
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_COLUMN_GET,
				 BT_MESH_SENSOR_MSG_MAXLEN_COLUMN_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_COLUMN_GET);
	net_buf_simple_add_le16(&msg, sensor->id);

	col_format = bt_mesh_sensor_column_format_get(sensor);
	if (col_format == NULL) {
		net_buf_simple_add_le16(&msg, column->index);
	} else {
		err = sensor_ch_encode(&msg, col_format, &column->sensor_value);
		if (err) {
			return err;
		}
	}

	struct series_data_rsp rsp_data = {
		.entries = rsp,
		.id = sensor->id,
		.count = 1,
		.col_start = col_format ? &column->sensor_value : NULL,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_COLUMN_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!rsp || err) {
		return err;
	}

	if (rsp_data.count == 0) {
		return -ENOENT;
	}

	return 0;
}

int bt_mesh_sensor_cli_series_entries_get(
	struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
	const struct bt_mesh_sensor_type *sensor,
	const union bt_mesh_sensor_column_key *range_start,
	const union bt_mesh_sensor_column_key *range_end,
	struct bt_mesh_sensor_series_entry *rsp, uint32_t *count)
{
	int err;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_SENSOR_OP_SERIES_GET,
				 BT_MESH_SENSOR_MSG_MAXLEN_SERIES_GET);
	bt_mesh_model_msg_init(&msg, BT_MESH_SENSOR_OP_SERIES_GET);

	net_buf_simple_add_le16(&msg, sensor->id);

	if (range_start && range_end) {
		const struct bt_mesh_sensor_format *col_format;

		col_format = bt_mesh_sensor_column_format_get(sensor);

		if (!col_format) {
			net_buf_simple_add_le16(&msg, range_start->index);
			net_buf_simple_add_le16(&msg, range_end->index);
		} else {
			err = sensor_ch_encode(&msg, col_format, &range_start->sensor_value);
			if (err) {
				return err;
			}

			err = sensor_ch_encode(&msg, col_format, &range_end->sensor_value);
			if (err) {
				return err;
			}
		}
	}

	struct series_data_rsp rsp_data = {
		.entries = rsp,
		.id = sensor->id,
		.count = count ? *count : 0,
	};

	struct bt_mesh_msg_rsp_ctx rsp_ctx = {
		.ack = &cli->ack_ctx,
		.op = BT_MESH_SENSOR_OP_SERIES_STATUS,
		.user_data = &rsp_data,
		.timeout = model_ackd_timeout_get(cli->model, ctx),
	};

	err = bt_mesh_msg_ackd_send(cli->model, ctx, &msg, rsp ? &rsp_ctx : NULL);
	if (!count || err) {
		return err;
	}

	*count = rsp_data.count;

	return 0;
}
