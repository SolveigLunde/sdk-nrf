/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <modem/nrf_modem_lib.h>
#include <net/fota_download.h>
#include <fota_download_util.h>

#include "fota.h"
#include "link.h"
#include "mosh_print.h"

static void on_modem_lib_dfu(int dfu_res, void *ctx)
{
	switch (dfu_res) {
	case NRF_MODEM_DFU_RESULT_OK:
		break;
	case NRF_MODEM_DFU_RESULT_UUID_ERROR:
	case NRF_MODEM_DFU_RESULT_AUTH_ERROR:
		mosh_error("FOTA: Modem firmware update failed, err: 0x%08x", dfu_res);
		mosh_error("FOTA: Modem is running previous firmware");
		break;
	case NRF_MODEM_DFU_RESULT_HARDWARE_ERROR:
	case NRF_MODEM_DFU_RESULT_INTERNAL_ERROR:
		mosh_error("FOTA: Fatal error, modem firmware update failed, err: 0x%08x", dfu_res);
		__ASSERT(false, "Modem firmware update failed on fatal error");
	case NRF_MODEM_DFU_RESULT_VOLTAGE_LOW:
		mosh_error("FOTA: Modem firmware update cancelled due to low voltage");
		mosh_error("FOTA: Please reboot once you have sufficient voltage for the DFU");
		__ASSERT(false, "Modem firmware update cancelled due to low voltage");
	default:
		/* Modem library initialization failed. */
		mosh_error("FOTA: Fatal error, could not initialize nrf_modem_lib, err: %d",
			   dfu_res);
		__ASSERT(false, "Could not initialize nrf_modem_lib");
	}
}

NRF_MODEM_LIB_ON_DFU_RES(fota_dfu_hook, on_modem_lib_dfu, NULL);

static void fota_update_apply(void)
{
	int err;
	int target;

	link_func_mode_set(LTE_LC_FUNC_MODE_OFFLINE, true);

	target = fota_download_target();
	if (target == DFU_TARGET_IMAGE_TYPE_MCUBOOT) {
		mosh_print("FOTA: Rebooting device to run new application firmware");

		sys_reboot(SYS_REBOOT_COLD);
	} else {
		mosh_print("FOTA: Applying modem firmware update...");

		err = fota_download_util_apply_update(target);
		if (err) {
			mosh_error("FOTA: Failed to apply modem firmware update, error: %d", err);
		} else {
			mosh_print("FOTA: Modem firmware update successful!");
		}

		link_func_mode_set(LTE_LC_FUNC_MODE_NORMAL, true);
	}
}

static const char *get_error_cause(enum fota_download_error_cause cause)
{
	switch (cause) {
	case FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED:
		return "download failed";
	case FOTA_DOWNLOAD_ERROR_CAUSE_INVALID_UPDATE:
		return "invalid update";
	default:
		return "unknown cause value";
	}
}

static void fota_download_callback(const struct fota_download_evt *evt)
{
	switch (evt->id) {
	case FOTA_DOWNLOAD_EVT_PROGRESS:
		mosh_print("FOTA: Progress %d%%", evt->progress);
		break;
	case FOTA_DOWNLOAD_EVT_FINISHED:
		mosh_print("FOTA: Download finished");
		fota_update_apply();
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_TIMEOUT:
		mosh_print("FOTA: Erasing reached timeout");
		/* Fall through, erasing continues. */
	case FOTA_DOWNLOAD_EVT_ERASE_PENDING:
		mosh_print("FOTA: Still erasing...");
		break;
	case FOTA_DOWNLOAD_EVT_ERASE_DONE:
		mosh_print("FOTA: Erasing finished");
		break;
	case FOTA_DOWNLOAD_EVT_ERROR:
		mosh_error("FOTA: Error, %s", get_error_cause(evt->cause));
		break;
	default:
		mosh_error("FOTA: Unknown event %d", evt->id);
		break;
	}
}

int fota_init(void)
{
	int err;

	err = fota_download_util_stream_init();
	if (err) {
		return err;
	}

	return fota_download_init(&fota_download_callback);
}

int fota_start(const char *host, const char *file)
{
	return fota_download_start(host, file, CONFIG_NRF_CLOUD_SEC_TAG, 0, 0);
}
