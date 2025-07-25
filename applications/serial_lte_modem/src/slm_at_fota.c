/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <stdint.h>
#include <stdio.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_delta_dfu.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/net/http/parser_url.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <net/fota_download.h>
#include <fota_download_util.h>
#include <dfu/dfu_target_full_modem.h>
#include <dfu/fmfu_fdev.h>
#include "slm_util.h"
#include "slm_settings.h"
#include "slm_at_host.h"
#include "slm_at_fota.h"

LOG_MODULE_REGISTER(slm_fota, CONFIG_SLM_LOG_LEVEL);

/* file_uri: scheme://hostname[:port]path[?parameters] */
#define FILE_URI_MAX	CONFIG_DOWNLOADER_MAX_FILENAME_SIZE
#define SCHEMA_HTTP     "http"
#define SCHEMA_HTTPS	"https"
#define URI_HOST_MAX	CONFIG_DOWNLOADER_MAX_HOSTNAME_SIZE
#define URI_SCHEMA_MAX	8
#define ERASE_POLL_TIME 2

/* Some features need fota_download update */
#define FOTA_FUTURE_FEATURE	0

enum slm_fota_operation {
	SLM_FOTA_STOP = 0,
	SLM_FOTA_START_APP = 1,
	SLM_FOTA_START_MFW = 2,
	SLM_FOTA_START_FULL_FOTA = 3,
	SLM_FOTA_PAUSE_RESUME = 4,
	SLM_FOTA_MFW_READ = 7,
	SLM_FOTA_ERASE_MFW = 9
};

bool slm_modem_full_fota;

uint8_t slm_fota_type = DFU_TARGET_IMAGE_TYPE_NONE;
enum fota_stage slm_fota_stage = FOTA_STAGE_INIT;
enum fota_status slm_fota_status;
int32_t slm_fota_info;

static char path[FILE_URI_MAX];
static char hostname[URI_HOST_MAX];

#if defined(CONFIG_SLM_FULL_FOTA)
/* Buffer used as temporary storage when downloading the modem firmware.
 */
#define FMFU_BUF_SIZE	32
static uint8_t fmfu_buf[FMFU_BUF_SIZE];

/* External flash device for full modem firmware storage */
#define FLASH_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(jedec_spi_nor)
static const struct device *flash_dev = DEVICE_DT_GET(FLASH_NODE);

/* dfu_target specific configurations */
static struct dfu_target_fmfu_fdev fdev;
static struct dfu_target_full_modem_params full_modem_fota_params;
#endif

static int do_fota_mfw_read(void)
{
	int err;
	size_t offset = 0;
	size_t area = 0;

	err = nrf_modem_delta_dfu_area(&area);
	if (err) {
		LOG_ERR("failed in delta dfu area: %d", err);
		return err;
	}

	err = nrf_modem_delta_dfu_offset(&offset);
	if (err) {
		LOG_ERR("failed in delta dfu offset: %d", err);
		return err;
	}

	rsp_send("\r\n#XFOTA: %d,%d\r\n", area, offset);

	return 0;
}

static int do_fota_erase_mfw(void)
{
	int err;
	size_t offset = 0;
	bool in_progress = false;

	err = nrf_modem_delta_dfu_offset(&offset);
	if (err) {
		if (err == NRF_MODEM_DELTA_DFU_ERASE_PENDING) {
			in_progress = true;
		} else {
			LOG_ERR("failed in delta dfu offset: %d", err);
			return err;
		}
	}

	if (offset != NRF_MODEM_DELTA_DFU_OFFSET_DIRTY && !in_progress) {
		/* No need to erase. */
		return 0;
	}

	if (!in_progress) {
		err = nrf_modem_delta_dfu_erase();
		if (err) {
			LOG_ERR("failed in delta dfu erase: %d", err);
			return err;
		}
	}

	int time_elapsed = 0;

	do {
		k_sleep(K_SECONDS(ERASE_POLL_TIME));
		err = nrf_modem_delta_dfu_offset(&offset);
		if (err != 0 && err != NRF_MODEM_DELTA_DFU_ERASE_PENDING) {
			LOG_ERR("failed in delta dfu offset: %d", err);
			return err;
		}
		if (err == 0 && offset == 0) {
			LOG_INF("Erase completed");
			break;
		}
		time_elapsed += ERASE_POLL_TIME;
	} while (time_elapsed < CONFIG_DFU_TARGET_MODEM_TIMEOUT);

	if (time_elapsed >= CONFIG_DFU_TARGET_MODEM_TIMEOUT) {
		LOG_WRN("Erase timeout");
		return -ETIME;
	}

	return 0;
}

static int do_fota_start(int op, const char *file_uri, int sec_tag,
			 uint8_t pdn_id, enum dfu_target_image_type type)
{
	int ret;
	struct http_parser_url parser = {
		/* UNINIT checker fix, assumed UF_SCHEMA existence */
		.field_set = 0
	};
	char schema[URI_SCHEMA_MAX];

	http_parser_url_init(&parser);
	ret = http_parser_parse_url(file_uri, strlen(file_uri), 0, &parser);
	if (ret) {
		LOG_ERR("Parse URL error");
		return -EINVAL;
	}

	/* Schema stores http/https information */
	memset(schema, 0x00, 8);
	if (parser.field_set & (1 << UF_SCHEMA)) {
		if (parser.field_data[UF_SCHEMA].len < URI_SCHEMA_MAX) {
			strncpy(schema, file_uri + parser.field_data[UF_SCHEMA].off,
				parser.field_data[UF_SCHEMA].len);
		} else {
			LOG_ERR("URL schema length %d too long, exceeds the max length of %d",
				parser.field_data[UF_SCHEMA].len, URI_SCHEMA_MAX);
			return -ENOMEM;
		}
	} else {
		LOG_ERR("Parse schema error");
		return -EINVAL;
	}

	/* Path includes folder and file information */
	/* This stores also the query data after folder and file description */
	memset(path, 0x00, FILE_URI_MAX);
	if (parser.field_set & (1 << UF_PATH)) {
		/* Remove the leading '/' as some HTTP servers don't like it */
		if ((strlen(file_uri) - parser.field_data[UF_PATH].off + 1) < FILE_URI_MAX) {
			strncpy(path, file_uri + parser.field_data[UF_PATH].off + 1,
				strlen(file_uri) - parser.field_data[UF_PATH].off - 1);
		} else {
			LOG_ERR("URL path length %d too long, exceeds the max length of %d",
					strlen(file_uri) - parser.field_data[UF_PATH].off - 1,
					FILE_URI_MAX);
			return -ENOMEM;
		}
	} else {
		LOG_ERR("Parse path error");
		return -EINVAL;
	}

	/* Stores everything before path (schema, hostname, port) */
	memset(hostname, 0x00, URI_HOST_MAX);
	if (parser.field_set & (1 << UF_HOST)) {
		if (parser.field_data[UF_PATH].off < URI_HOST_MAX) {
			strncpy(hostname, file_uri, parser.field_data[UF_PATH].off);
		} else {
			LOG_ERR("URL host name length %d too long, exceeds the max length of %d",
					parser.field_data[UF_PATH].off, URI_HOST_MAX);
			return -ENOMEM;
		}
	} else {
		LOG_ERR("Parse host error");
		return -EINVAL;
	}

	/* start HTTP(S) FOTA */
	if (slm_util_casecmp(schema, SCHEMA_HTTPS)) {
		if (sec_tag == INVALID_SEC_TAG) {
			LOG_ERR("Missing sec_tag");
			return -EINVAL;
		}
		ret = fota_download_start_with_image_type(hostname, path, sec_tag, pdn_id, 0, type);
	} else if (slm_util_casecmp(schema, SCHEMA_HTTP)) {
		ret = fota_download_start_with_image_type(hostname, path, -1, pdn_id, 0, type);
	} else {
		ret = -EINVAL;
	}
	/* Send an URC if failed to start */
	if (ret) {
		rsp_send("\r\n#XFOTA: %d,%d,%d\r\n", FOTA_STAGE_DOWNLOAD,
			FOTA_STATUS_ERROR, ret);
	}

	slm_fota_init_state();
	slm_fota_type = type;

	return ret;
}

static void fota_dl_handler(const struct fota_download_evt *evt)
{
	switch (evt->id) {
	case FOTA_DOWNLOAD_EVT_PROGRESS:
		slm_fota_stage = FOTA_STAGE_DOWNLOAD;
		slm_fota_status = FOTA_STATUS_OK;
		slm_fota_info = evt->progress;
		rsp_send("\r\n#XFOTA: %d,%d,%d\r\n",
			slm_fota_stage, slm_fota_status, slm_fota_info);
		break;
	case FOTA_DOWNLOAD_EVT_FINISHED:
		slm_fota_stage = FOTA_STAGE_ACTIVATE;
		slm_fota_info = 0;
		slm_modem_full_fota = (slm_fota_type == DFU_TARGET_IMAGE_TYPE_FULL_MODEM);
		/* Save, in case reboot by reset */
		slm_settings_fota_save();
		rsp_send("\r\n#XFOTA: %d,%d\r\n", slm_fota_stage, slm_fota_status);
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_TIMEOUT:
		LOG_INF("Erasure timeout reached. Erasure continues.");
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_PENDING:
		slm_fota_stage = FOTA_STAGE_DOWNLOAD_ERASE_PENDING;
		rsp_send("\r\n#XFOTA: %d,%d\r\n", slm_fota_stage, slm_fota_status);
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_DONE:
		rsp_send("\r\n#XFOTA: %d,%d\r\n", FOTA_STAGE_DOWNLOAD_ERASED, slm_fota_status);
		/* Back to init now that the erasure is complete so that potential pre-start
		 * error codes are printed with the same stage than if there had been no erasure.
		 */
		slm_fota_stage = FOTA_STAGE_INIT;
		break;
	case FOTA_DOWNLOAD_EVT_ERROR:
		slm_fota_status = FOTA_STATUS_ERROR;
		slm_fota_info = evt->cause;
		rsp_send("\r\n#XFOTA: %d,%d,%d\r\n",
			slm_fota_stage, slm_fota_status, slm_fota_info);
		/* FOTA session terminated */
		slm_fota_init_state();
		break;
	case FOTA_DOWNLOAD_EVT_CANCELLED:
		slm_fota_status = FOTA_STATUS_CANCELLED;
		slm_fota_info = 0;
		rsp_send("\r\n#XFOTA: %d,%d\r\n", slm_fota_stage, slm_fota_status);
		/* FOTA session terminated */
		slm_fota_init_state();
		break;

	default:
		return;
	}
}

SLM_AT_CMD_CUSTOM(xfota, "AT#XFOTA", handle_at_fota);
static int handle_at_fota(enum at_parser_cmd_type cmd_type, struct at_parser *parser,
			  uint32_t param_count)
{
	int err = -EINVAL;
	uint16_t op;
#if FOTA_FUTURE_FEATURE
	static bool paused;
#endif

	switch (cmd_type) {
	case AT_PARSER_CMD_TYPE_SET:
		err = at_parser_num_get(parser, 1, &op);
		if (err < 0) {
			return err;
		}
		switch (op) {
		case SLM_FOTA_STOP:
			err = fota_download_cancel();
			break;
		case SLM_FOTA_START_APP:
		case SLM_FOTA_START_MFW:
#if defined(CONFIG_SLM_FULL_FOTA)
		case SLM_FOTA_START_FULL_FOTA:
#endif
			char uri[FILE_URI_MAX];
			uint16_t pdn_id;
			int size = FILE_URI_MAX;
			sec_tag_t sec_tag = INVALID_SEC_TAG;
			enum dfu_target_image_type type;

			err = util_string_get(parser, 2, uri, &size);
			if (err) {
				return err;
			}
			if (param_count > 3) {
				at_parser_num_get(parser, 3, &sec_tag);
			}
			if (op == SLM_FOTA_START_APP) {
				type = DFU_TARGET_IMAGE_TYPE_MCUBOOT;
			}
#if defined(CONFIG_SLM_FULL_FOTA)
			else if (op == SLM_FOTA_START_FULL_FOTA)  {
				fdev.dev = flash_dev;
				fdev.size = DT_PROP(FLASH_NODE, size) / 8;
				full_modem_fota_params.buf = fmfu_buf;
				full_modem_fota_params.len = sizeof(fmfu_buf);
				full_modem_fota_params.dev = &fdev;

				if (!device_is_ready(flash_dev)) {
					LOG_ERR("Flash device %s not ready", flash_dev->name);
					return -ENXIO;
				}

				err = dfu_target_full_modem_cfg(&full_modem_fota_params);
				if (err != 0 && err != -EALREADY) {
					LOG_ERR("dfu_target_full_modem_cfg failed: %d", err);
					return err;
				}

				type = DFU_TARGET_IMAGE_TYPE_FULL_MODEM;
			}
#endif
			else {
				type = DFU_TARGET_IMAGE_TYPE_MODEM_DELTA;
			}
			if (param_count > 4) {
				at_parser_num_get(parser, 4, &pdn_id);
				err = do_fota_start(op, uri, sec_tag, pdn_id, type);
			} else {
				err = do_fota_start(op, uri, sec_tag, 0, type);
			}
			break;
#if FOTA_FUTURE_FEATURE
		case SLM_FOTA_PAUSE_RESUME:
			if (paused) {
				fota_download_resume();
				paused = false;
			} else {
				fota_download_pause();
				paused = true;
			}
			err = 0;
			break;
#endif
		case SLM_FOTA_MFW_READ:
			err = do_fota_mfw_read();
			break;
		case SLM_FOTA_ERASE_MFW:
			err = do_fota_erase_mfw();
			break;
		default:
			err = -EINVAL;
			break;
		}
		break;

	case AT_PARSER_CMD_TYPE_TEST:
#if defined(CONFIG_SLM_FULL_FOTA)
		rsp_send("\r\n#XFOTA: (%d,%d,%d,%d,%d,%d)[,<file_url>[,<sec_tag>[,<pdn_id>]]]\r\n",
			SLM_FOTA_STOP, SLM_FOTA_START_APP, SLM_FOTA_START_MFW,
			SLM_FOTA_MFW_READ, SLM_FOTA_ERASE_MFW, SLM_FOTA_START_FULL_FOTA);
#else
		rsp_send("\r\n#XFOTA: (%d,%d,%d,%d,%d)[,<file_url>[,<sec_tag>[,<pdn_id>]]]\r\n",
			SLM_FOTA_STOP, SLM_FOTA_START_APP, SLM_FOTA_START_MFW,
			SLM_FOTA_MFW_READ, SLM_FOTA_ERASE_MFW);
#endif
		err = 0;
		break;

	default:
		break;
	}

	return err;
}

int slm_at_fota_init(void)
{
	return fota_download_init(fota_dl_handler);
}

int slm_at_fota_uninit(void)
{
	return 0;
}

void slm_fota_init_state(void)
{
	slm_modem_full_fota = false;
	slm_fota_type = DFU_TARGET_IMAGE_TYPE_NONE;
	slm_fota_stage = FOTA_STAGE_INIT;
	slm_fota_status = FOTA_STATUS_OK;
	slm_fota_info = 0;
}

#if defined(CONFIG_LWM2M_CARRIER)
static K_SEM_DEFINE(carrier_app_fota_status, 0, 1);
static bool carrier_app_fota_success;

bool lwm2m_os_dfu_application_update_validate(void)
{
	/* Wait for the application FOTA status to be checked by the main thread. This can
	 * trigger an AT notification, so the UART backend must also be initialized.
	 */
	if (k_sem_take(&carrier_app_fota_status, K_SECONDS(10)) != 0) {
		return false;
	}

	return carrier_app_fota_success;
}
#endif /* CONFIG_LWM2M_CARRIER */

void slm_fota_post_process(void)
{
#if defined(CONFIG_LWM2M_CARRIER)
	if ((slm_fota_type == DFU_TARGET_IMAGE_TYPE_MCUBOOT) &&
	    (slm_fota_status == FOTA_STATUS_OK) &&
	    (slm_fota_stage == FOTA_STAGE_COMPLETE)) {
		carrier_app_fota_success = true;
	}

	k_sem_give(&carrier_app_fota_status);
#endif /* CONFIG_LWM2M_CARRIER */
	if (slm_fota_stage != FOTA_STAGE_COMPLETE && slm_fota_stage != FOTA_STAGE_ACTIVATE) {
		return;
	}
	LOG_INF("FOTA result %d,%d,%d", slm_fota_stage, slm_fota_status, slm_fota_info);

	if (slm_fota_status == FOTA_STATUS_OK) {
		rsp_send("\r\n#XFOTA: %d,%d\r\n", slm_fota_stage, slm_fota_status);
	} else {
		rsp_send("\r\n#XFOTA: %d,%d,%d\r\n", slm_fota_stage, slm_fota_status,
			slm_fota_info);
	}

	slm_fota_init_state();
	slm_settings_fota_save();
}

#if defined(CONFIG_SLM_FULL_FOTA)

FUNC_NORETURN static void handle_full_fota_activation_fail(int ret)
{
	int err;

	/* Send the result notification and terminate the FOTA session. */
	slm_fota_status = FOTA_STATUS_ERROR;
	slm_fota_info = ret;
	slm_fota_post_process();

	LOG_ERR("Modem firmware activation failed, error: %d", ret);

	/* Extenal flash needs to be erased and internal counters cleared */
	err = dfu_target_reset();
	if (err != 0)
		LOG_ERR("dfu_target_reset() failed: %d", err);
	else
		LOG_INF("External flash erase succeeded");

	LOG_WRN("Rebooting...");
	LOG_PANIC();
	sys_reboot(SYS_REBOOT_COLD);
}

void slm_finish_modem_full_fota(void)
{
	int err;

	/* All erroneous steps in activation stage are
	 * considered fatal and the device is reset.
	 */
	slm_fota_stage = FOTA_STAGE_COMPLETE;
	LOG_INF("Applying full modem firmware update from external flash");

	err = nrf_modem_lib_bootloader_init();
	if (err != 0) {
		LOG_ERR("nrf_modem_lib_bootloader_init() failed: %d", err);
		handle_full_fota_activation_fail(err);
	}

	/* Loading data from external flash to modem's flash. */
	err = fmfu_fdev_load(fmfu_buf, sizeof(fmfu_buf), flash_dev, 0);
	if (err != 0) {
		LOG_ERR("fmfu_fdev_load failed: %d", err);
		handle_full_fota_activation_fail(err);
	}

	err = nrf_modem_lib_shutdown();
	if (err != 0) {
		LOG_ERR("nrf_modem_lib_shutdown() failed: %d", err);
		handle_full_fota_activation_fail(err);
	}

	/* Reset dfu_target to a pristine state, otherwise it has been observed to
	 * fail when attempting a second full MFW update without application reset.
	 */
	err = fota_download_util_image_reset(DFU_TARGET_IMAGE_TYPE_FULL_MODEM);
	if (err) {
		LOG_ERR("fota_download_util_image_reset() failed: %d\n", err);
	}

	slm_fota_status = FOTA_STATUS_OK;
	slm_fota_info = 0;
	LOG_INF("Full modem firmware update complete.");
}

#endif /* CONFIG_SLM_FULL_FOTA */
