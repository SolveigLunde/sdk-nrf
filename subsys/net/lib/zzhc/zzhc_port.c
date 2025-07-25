/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zzhc_port, CONFIG_ZZHC_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/sys/base64.h>
#include <zephyr/data/json.h>
#include <modem/at_parser.h>
#include "zzhc_internal.h"

#define AT_PARAMS_MAX     11        /** Max. # of AT-params to parse */
#define FICR_REV_OFFSET   0x134

/** Cached storage of ICCID. */
static char ueiccid[ICCID_LEN];

/** Pointer to settings_handler. */
static struct settings_handler settings_fn = {0};

/** Name of settings */
static char const settings_name[] = "zzhc";

/**@brief Structure to store decoded json object. */
struct rsp_obj {
	const char *res_code;   /**< json field name: "resultCode". */
	const char *res_desc;   /**< json field name: "resultDesc". */
};

/**@brief Descriptor for json parser. */
static const struct json_obj_descr rsp_desc[] = {
	JSON_OBJ_DESCR_PRIM_NAMED(struct rsp_obj, "resultCode", res_code,
		JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM_NAMED(struct rsp_obj, "resultDesc", res_desc,
		JSON_TOK_STRING),
};

int zzhc_load_iccid(char *iccid_buf, int buf_len)
{
	memcpy(iccid_buf, ueiccid, buf_len);
	return 0;
}

int zzhc_base64(unsigned char *dst, size_t dlen, const unsigned char *src,
	size_t slen)
{
	int i = 0, rc;

	if (dlen == 0 || slen == 0) {
		return 0;
	}

	rc = base64_encode(dst, dlen, &i, src, slen);
	return (rc == 0) ? i : 0;
}

/**@brief Callback when settings_load() is called. */
static int settings_set(const char *key, size_t len,
		settings_read_cb read_cb, void *cb_arg)
{
	int rc;

	if (strcmp(key, "iccid") == 0) {
		rc = read_cb(cb_arg, ueiccid, ICCID_LEN);
		if (rc < 0) {
			return rc;
		}
	}

	return 0;
}

int zzhc_ext_init(struct zzhc *ctx)
{
	int rc;

	memset(ueiccid, 0, ICCID_LEN);

	ctx->sem = zzhc_malloc(sizeof(struct k_sem));
	if (ctx->sem == NULL) {
		return -ENOBUFS;
	}

	/* Initialize settings (run-once). */
	if (settings_fn.name == NULL) {
		rc = settings_subsys_init();
		if (rc) {
			LOG_DBG("settings_subsys_init() = %d", rc);
			return rc;
		}

		settings_fn.name  = (char *)settings_name;
		settings_fn.h_set = settings_set;
		rc = settings_register(&settings_fn);
		if (rc) {
			LOG_DBG("settings_register() = %d", rc);
			return rc;
		}
	}

	/* Read from storage. */
	rc = settings_load();
	if (rc) {
		return rc;
	}

	return 0;
}

void zzhc_ext_uninit(struct zzhc *ctx)
{
	zzhc_free(ctx->sem);
}

bool zzhc_check_http_payload(struct zzhc *ctx)
{
	int rc;
	bool res_code, res_desc = false;
	struct rsp_obj obj;

	LOG_DBG("Received pkt:\n[%s]", ctx->http_pkt);

	rc = json_obj_parse(ctx->http_pkt, ctx->http_pkt_len, rsp_desc,
		ARRAY_SIZE(rsp_desc), &obj);
	if (rc != 0x03) {
		LOG_DBG("json_obj_parse() = 0x%08x", rc);
		return false;
	}

	LOG_DBG("Decoded: obj.res_code = %s, obj.res_desc = %s.",
		obj.res_code, obj.res_desc);

	res_code = (atoi(obj.res_code) == 0) ? true : false;
	if (strcmp(obj.res_desc, "Success") == 0) {
		res_desc = true;
	}

	return (res_code && res_desc);
}

int zzhc_get_at_param_short(struct zzhc *ctx, const char *data, int idx)
{
	int rc;
	uint16_t evt = 0;
	struct at_parser parser;
	size_t count = 0;

	rc = at_parser_init(&parser, data);
	if (rc < 0) {
		LOG_DBG("at_parser_init()=%d", rc);
		return rc;
	}

	rc = at_parser_cmd_count_get(&parser, &count);
	if (rc < 0 || count < idx + 2) {
		LOG_DBG("at_parser_cmd_count_get()=%d, count=%d", rc, count);
		return rc;
	}

	rc = at_parser_num_get(&parser, idx + 1, &evt);
	if (rc < 0) {
		LOG_DBG("at_parser_num_get()=%d, evt=%d", rc, evt);
		return rc;
	}

	return evt;
}
