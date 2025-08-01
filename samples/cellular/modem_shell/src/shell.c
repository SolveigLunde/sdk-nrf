/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/shell/shell.h>

#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

#include "mosh_print.h"
#include "link_shell.h"

#if defined(CONFIG_MOSH_IPERF3)
#include <zephyr/posix/sys/select.h>
#include <iperf_api.h>
#endif

#if defined(CONFIG_MOSH_CURL)
#include <nrf_curl.h>
#endif

#if defined(CONFIG_LWM2M_CARRIER)
#include <modem/pdn.h>
#include <lwm2m_carrier.h>
#include "link.h"
#include "link_settings.h"
#endif /* CONFIG_LWM2M_CARRIER */

extern struct k_poll_signal mosh_signal;

#if defined(CONFIG_LWM2M_CARRIER)
void lwm2m_handle_error(const lwm2m_carrier_event_t *evt)
{
	const lwm2m_carrier_event_error_t *err = evt->data.error;

	static const char * const strerr[] = {
		[LWM2M_CARRIER_ERROR_NO_ERROR] =
			"No error",
		[LWM2M_CARRIER_ERROR_BOOTSTRAP] =
			"Bootstrap error",
		[LWM2M_CARRIER_ERROR_LTE_LINK_UP_FAIL] =
			"Failed to connect to the LTE network",
		[LWM2M_CARRIER_ERROR_LTE_LINK_DOWN_FAIL] =
			"Failed to disconnect from the LTE network",
		[LWM2M_CARRIER_ERROR_FOTA_FAIL] =
			"Modem firmware update failed",
		[LWM2M_CARRIER_ERROR_CONFIGURATION] =
			"Configuration failure",
		[LWM2M_CARRIER_ERROR_INIT] =
			"Initialization failure",
		[LWM2M_CARRIER_ERROR_CONNECT] =
			"Connection failure",
	};

	mosh_error("%s, reason %d\n", strerr[err->type], err->value);
}

void lwm2m_print_deferred(const lwm2m_carrier_event_t *evt)
{
	const lwm2m_carrier_event_deferred_t *def = evt->data.deferred;

	static const char *const strdef[] = {
		[LWM2M_CARRIER_DEFERRED_NO_REASON] =
			"No reason given",
		[LWM2M_CARRIER_DEFERRED_PDN_ACTIVATE] =
			"Failed to activate PDN",
		[LWM2M_CARRIER_DEFERRED_BOOTSTRAP_NO_ROUTE] =
			"No route to bootstrap server",
		[LWM2M_CARRIER_DEFERRED_BOOTSTRAP_CONNECT] =
			"Failed to connect to bootstrap server",
		[LWM2M_CARRIER_DEFERRED_BOOTSTRAP_SEQUENCE] =
			"Bootstrap sequence not completed",
		[LWM2M_CARRIER_DEFERRED_SERVER_NO_ROUTE] =
			"No route to server",
		[LWM2M_CARRIER_DEFERRED_SERVER_CONNECT] =
			"Failed to connect to server",
		[LWM2M_CARRIER_DEFERRED_SERVER_REGISTRATION] =
			"Server registration sequence not completed",
		[LWM2M_CARRIER_DEFERRED_SERVICE_UNAVAILABLE] =
			"Server in maintenance mode",
		[LWM2M_CARRIER_DEFERRED_SIM_MSISDN] =
			"Waiting for SIM MSISDN",
	};

	mosh_error("Reason: %s, timeout: %d seconds\n", strdef[def->reason], def->timeout);
}

void lwm2m_print_app_data(const lwm2m_carrier_event_t *evt)
{
	const uint16_t *path = evt->data.app_data->path;
	const uint8_t event_type = evt->data.app_data->type;

	static const char *const app_data_event_types[] = {
		[LWM2M_CARRIER_APP_DATA_EVENT_DATA_WRITE] = "DATA_WRITE",
		[LWM2M_CARRIER_APP_DATA_EVENT_OBSERVE_START] = "OBSERVE_START",
		[LWM2M_CARRIER_APP_DATA_EVENT_OBSERVE_STOP] = "OBSERVE_STOP",
	};

	char path_string[sizeof("/65535/65535/65535/65535")];
	uint32_t offset = 0;

	for (int i = 0; i < evt->data.app_data->path_len; i++) {
		offset += snprintf(&path_string[offset], sizeof(path_string) - offset,
				   "/%hu", path[i]);
		if (offset >= sizeof(path_string)) {
			break;
		}
	}

	mosh_print("App data event type: %s, path: %s\n",
		app_data_event_types[event_type], path_string);
}

int lwm2m_carrier_event_handler(const lwm2m_carrier_event_t *event)
{
	int err = 0;

	switch (event->type) {
	case LWM2M_CARRIER_EVENT_LTE_LINK_UP:
		mosh_print("LwM2M carrier event: request LTE Link up");
		link_func_mode_set(LTE_LC_FUNC_MODE_NORMAL,
				   link_sett_is_normal_mode_autoconn_rel14_used());
		return 0;
	case LWM2M_CARRIER_EVENT_LTE_LINK_DOWN:
		mosh_print("LwM2M carrier event: request LTE Link down");
		link_func_mode_set(LTE_LC_FUNC_MODE_OFFLINE, false);
		break;
	case LWM2M_CARRIER_EVENT_LTE_POWER_OFF:
		mosh_print("LwM2M carrier event: request LTE Power off");
		link_func_mode_set(LTE_LC_FUNC_MODE_POWER_OFF, false);
		break;
	case LWM2M_CARRIER_EVENT_BOOTSTRAPPED:
		mosh_print("LwM2M carrier event: bootstrapped");
		break;
	case LWM2M_CARRIER_EVENT_REGISTERED:
		mosh_print("LwM2M carrier event: registered");
		break;
	case LWM2M_CARRIER_EVENT_DEFERRED:
		mosh_print("LwM2M carrier event: deferred");
		lwm2m_print_deferred(event);
		break;
	case LWM2M_CARRIER_EVENT_FOTA_START:
		mosh_print("LwM2M carrier event: fota start");
		break;
	case LWM2M_CARRIER_EVENT_FOTA_SUCCESS:
		mosh_print("LwM2M carrier event: fota success");
		break;
	case LWM2M_CARRIER_EVENT_REBOOT:
		mosh_print("LwM2M carrier event: reboot");
		break;
	case LWM2M_CARRIER_EVENT_MODEM_DOMAIN:
		mosh_print("LwM2M carrier event: modem domain");
		break;
	case LWM2M_CARRIER_EVENT_APP_DATA:
		mosh_print("LwM2M carrier event: app data");
		lwm2m_print_app_data(event);
		break;
	case LWM2M_CARRIER_EVENT_MODEM_INIT:
		mosh_print("LwM2M carrier event: modem init");
		err = nrf_modem_lib_init();
		break;
	case LWM2M_CARRIER_EVENT_MODEM_SHUTDOWN:
		mosh_print("LwM2M carrier event: modem shutdown");
		err = nrf_modem_lib_shutdown();
		break;
	case LWM2M_CARRIER_EVENT_ERROR_CODE_RESET:
		mosh_print("LWM2M_CARRIER_EVENT_ERROR_CODE_RESET");
		break;
	case LWM2M_CARRIER_EVENT_ERROR:
		mosh_print("LwM2M carrier event: error");
		lwm2m_handle_error(event);
		break;
	}

	return err;
}
#endif

int sleep_shell(const struct shell *shell, size_t argc, char **argv)
{
	long sleep_duration = strtol(argv[1], NULL, 10);

	if (sleep_duration > 0) {
		k_sleep(K_SECONDS(sleep_duration));
		return 0;
	}
	mosh_print("sleep: duration must be greater than zero");

	return -EINVAL;
}

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
extern struct sys_heap _system_heap;

int heap_shell(const struct shell *shell, size_t argc, char **argv)
{
	int err;
	struct sys_memory_stats kernel_stats;

	err = sys_heap_runtime_stats_get(&_system_heap, &kernel_stats);
	if (err) {
		mosh_error("heap: failed to read kernel heap statistics, error: %d", err);
	} else {
		mosh_print("kernel heap statistics:");
		mosh_print("free:           %6d", kernel_stats.free_bytes);
		mosh_print("allocated:      %6d", kernel_stats.allocated_bytes);
		mosh_print("max. allocated: %6d\n", kernel_stats.max_allocated_bytes);
	}

	return 0;
}
#endif /* CONFIG_SYS_HEAP_RUNTIME_STATS */

int version_shell(const struct shell *shell, size_t argc, char **argv)
{
	mosh_print_version_info();

	return 0;
}

#if defined(CONFIG_MOSH_IPERF3)
static int cmd_iperf3(const struct shell *shell, size_t argc, char **argv)
{
	(void)iperf_main(argc, argv, NULL, 0, &mosh_signal);
	return 0;
}
#endif

#if defined(CONFIG_MOSH_CURL)
static int cmd_curl(const struct shell *shell, size_t argc, char **argv)
{
	(void)curl_tool_main(argc, argv, &mosh_signal);
	mosh_print("\nDONE");
	return 0;
}
SHELL_CMD_REGISTER(curl, NULL, "For curl usage, just type \"curl --manual\"", cmd_curl);
#endif

#if defined(CONFIG_MOSH_IPERF3)
SHELL_CMD_REGISTER(iperf3, NULL, "For iperf3 usage, just type \"iperf3 --manual\"", cmd_iperf3);
#endif

SHELL_CMD_ARG_REGISTER(sleep, NULL,
	"Sleep for n seconds.",
	sleep_shell, 2, 0);

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
SHELL_CMD_ARG_REGISTER(heap, NULL,
	"Print heap usage statistics.",
	heap_shell, 1, 0);
#endif

SHELL_CMD_ARG_REGISTER(version, NULL,
	"Print application version information.",
	version_shell, 1, 0);
