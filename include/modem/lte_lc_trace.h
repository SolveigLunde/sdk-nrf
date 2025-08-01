/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef LTE_LC_TRACE_H__
#define LTE_LC_TRACE_H__

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LTE_LC_TRACE(type)							\
	IF_ENABLED(CONFIG_LTE_LC_TRACE,						\
		  (lte_lc_trace_capture(type)))

enum lte_lc_trace_type {
	LTE_LC_TRACE_FUNC_MODE_NORMAL,
	LTE_LC_TRACE_FUNC_MODE_ACTIVATE_LTE,
	LTE_LC_TRACE_FUNC_MODE_POWER_OFF,
	LTE_LC_TRACE_FUNC_MODE_RX_ONLY,
	LTE_LC_TRACE_FUNC_MODE_OFFLINE,
	LTE_LC_TRACE_FUNC_MODE_DEACTIVATE_LTE,
	LTE_LC_TRACE_FUNC_MODE_DEACTIVATE_GNSS,
	LTE_LC_TRACE_FUNC_MODE_ACTIVATE_GNSS,
	LTE_LC_TRACE_FUNC_MODE_DEACTIVATE_UICC,
	LTE_LC_TRACE_FUNC_MODE_ACTIVATE_UICC,
	LTE_LC_TRACE_FUNC_MODE_OFFLINE_UICC_ON,
	LTE_LC_TRACE_NW_REG_NOT_REGISTERED,
	LTE_LC_TRACE_NW_REG_REGISTERED_HOME,
	LTE_LC_TRACE_NW_REG_SEARCHING,
	LTE_LC_TRACE_NW_REG_REGISTRATION_DENIED,
	LTE_LC_TRACE_NW_REG_UNKNOWN,
	LTE_LC_TRACE_NW_REG_REGISTERED_ROAMING,
	LTE_LC_TRACE_NW_REG_REGISTERED_EMERGENCY,
	LTE_LC_TRACE_NW_REG_UICC_FAIL,
	LTE_LC_TRACE_RRC_CONNECTED,
	LTE_LC_TRACE_RRC_IDLE,
	LTE_LC_TRACE_LTE_MODE_UPDATE_NONE,
	LTE_LC_TRACE_LTE_MODE_UPDATE_LTEM,
	LTE_LC_TRACE_LTE_MODE_UPDATE_NBIOT,
	LTE_LC_TRACE_MODEM_SLEEP_EXIT,
	LTE_LC_TRACE_MODEM_SLEEP_ENTER,
};

typedef void(*lte_lc_trace_handler_t)(enum lte_lc_trace_type type);

void lte_lc_trace_handler_set(lte_lc_trace_handler_t handler);

void lte_lc_trace_capture(enum lte_lc_trace_type type);

#ifdef __cplusplus
}
#endif

#endif /* LTE_LC_TRACE_H__ */
