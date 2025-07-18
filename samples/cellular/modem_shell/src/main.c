/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/shell/shell.h>
#if defined(CONFIG_SHELL_BACKEND_SERIAL)
#include <zephyr/shell/shell_uart.h>
#else
#include <zephyr/shell/shell_rtt.h>
#endif

#include <nrf_modem.h>
#include <helpers/nrfx_reset_reason.h>
#include <modem/nrf_modem_lib.h>
#include <modem/nrf_modem_lib_trace.h>
#include <modem/modem_info.h>
#include <modem/lte_lc.h>
#include <net/nrf_cloud_os.h>
#include <dk_buttons_and_leds.h>

#include "mosh_defines.h"
#include "mosh_print.h"
#include "link.h"
#include "uart_shell.h"

#if defined(CONFIG_MOSH_PPP)
#include "ppp_ctrl.h"
#endif

#if defined(CONFIG_MOSH_GNSS)
#include "gnss.h"
#endif

#if defined(CONFIG_MOSH_FOTA)
#include "fota.h"
#endif
#if defined(CONFIG_MOSH_WORKER_THREADS)
#include "th_ctrl.h"
#endif

#if defined(CONFIG_MOSH_LOCATION)
#include "location_shell.h"
#endif

#if defined(CONFIG_MOSH_STARTUP_CMDS)
#include "startup_cmd_ctrl.h"
#endif

#if defined(CONFIG_MOSH_AT_CMD_MODE)
#include "at_cmd_mode.h"
#include "at_cmd_mode_sett.h"
#endif

BUILD_ASSERT(IS_ENABLED(CONFIG_SHELL_BACKEND_SERIAL) || IS_ENABLED(CONFIG_SHELL_BACKEND_RTT),
	     "CONFIG_SHELL_BACKEND_SERIAL or CONFIG_SHELL_BACKEND_RTT shell backend must be "
	     "enabled");

/***** Work queue and work item definitions *****/

#define MOSH_COMMON_WORKQ_PRIORITY CONFIG_MOSH_COMMON_WORKQUEUE_PRIORITY
K_THREAD_STACK_DEFINE(mosh_common_workq_stack, CONFIG_MOSH_COMMON_WORKQUEUE_STACK_SIZE);
struct k_work_q mosh_common_work_q;

/* Global variables */
const struct shell *mosh_shell;
struct k_poll_signal mosh_signal;

char mosh_at_resp_buf[MOSH_AT_CMD_RESPONSE_MAX_LEN];
K_MUTEX_DEFINE(mosh_at_resp_buf_mutex);

static const char *modem_crash_reason_get(uint32_t reason)
{
	switch (reason) {
	case NRF_MODEM_FAULT_UNDEFINED:
		return "Undefined fault";

	case NRF_MODEM_FAULT_HW_WD_RESET:
		return "HW WD reset";

	case NRF_MODEM_FAULT_HARDFAULT:
		return "Hard fault";

	case NRF_MODEM_FAULT_MEM_MANAGE:
		return "Memory management fault";

	case NRF_MODEM_FAULT_BUS:
		return "Bus fault";

	case NRF_MODEM_FAULT_USAGE:
		return "Usage fault";

	case NRF_MODEM_FAULT_SECURE_RESET:
		return "Secure control reset";

	case NRF_MODEM_FAULT_PANIC_DOUBLE:
		return "Error handler crash";

	case NRF_MODEM_FAULT_PANIC_RESET_LOOP:
		return "Reset loop";

	case NRF_MODEM_FAULT_ASSERT:
		return "Assert";

	case NRF_MODEM_FAULT_PANIC:
		return "Unconditional SW reset";

	case NRF_MODEM_FAULT_FLASH_ERASE:
		return "Flash erase fault";

	case NRF_MODEM_FAULT_FLASH_WRITE:
		return "Flash write fault";

	case NRF_MODEM_FAULT_POFWARN:
		return "Undervoltage fault";

	case NRF_MODEM_FAULT_THWARN:
		return "Overtemperature fault";

	default:
		return "Unknown reason";
	}
}

void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	printk("Modem crash reason: 0x%x (%s), PC: 0x%x\n",
		fault_info->reason,
		modem_crash_reason_get(fault_info->reason),
		fault_info->program_counter);

	__ASSERT(false, "Modem crash detected, halting application execution");
}

#if defined(CONFIG_NRF_MODEM_LIB_SHELL_TRACE)
void nrf_modem_lib_trace_callback(enum nrf_modem_lib_trace_event evt)
{
	if (evt == NRF_MODEM_LIB_TRACE_EVT_FULL) {
		mosh_warn("Modem trace storage is full.");
		mosh_print("It is recommended to stop modem tracing before sending.");
		mosh_print("Send or clear modem traces before re-starting.");
	}
}
#endif

static void reset_reason_str_get(char *str, uint32_t reason)
{
	size_t len;

	*str = '\0';

	if (reason & NRFX_RESET_REASON_RESETPIN_MASK) {
		(void)strcat(str, "PIN reset | ");
	}
	if (reason & NRFX_RESET_REASON_DOG_MASK) {
		(void)strcat(str, "watchdog | ");
	}
	if (reason & NRFX_RESET_REASON_OFF_MASK) {
		(void)strcat(str, "wakeup from power-off | ");
	}
	if (reason & NRFX_RESET_REASON_DIF_MASK) {
		(void)strcat(str, "debug interface wakeup | ");
	}
	if (reason & NRFX_RESET_REASON_SREQ_MASK) {
		(void)strcat(str, "software | ");
	}
	if (reason & NRFX_RESET_REASON_LOCKUP_MASK) {
		(void)strcat(str, "CPU lockup | ");
	}
	if (reason & NRFX_RESET_REASON_CTRLAP_MASK) {
		(void)strcat(str, "control access port | ");
	}

	len = strlen(str);
	if (len == 0) {
		(void)strcpy(str, "power-on reset");
	} else {
		str[len - 3] = '\0';
	}
}

static void mosh_print_reset_reason(void)
{
	uint32_t reset_reason;
	char reset_reason_str[128];

	/* Read RESETREAS register value and clear current reset reason(s). */
	reset_reason = nrfx_reset_reason_get();
	nrfx_reset_reason_clear(reset_reason);

	reset_reason_str_get(reset_reason_str, reset_reason);

	printk("\nReset reason: %s\n", reset_reason_str);
}

#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	if (has_changed & button_states & DK_BTN1_MSK) {
		mosh_print("Button 1 pressed - raising a kill signal");
		k_poll_signal_raise(&mosh_signal, MOSH_SIGNAL_KILL);
#if defined(CONFIG_MOSH_WORKER_THREADS)
		th_ctrl_kill_em_all();
#endif
	} else if (has_changed & ~button_states & DK_BTN1_MSK) {
		mosh_print("Button 1 released - resetting a kill signal");
		k_poll_signal_reset(&mosh_signal);
	}

	if (has_changed & button_states & DK_BTN2_MSK) {
		/* printk() used instead of mosh_print() which returns before the printing is
		 * actually finished and UART gets shut down in the meantime causing a jam.
		 */
		printk("\nButton 2 pressed, toggling UART power state\n");
		uart_toggle_power_state();
	}
}
#endif

int main(void)
{
	int err;
	struct k_work_queue_config cfg = {
		.name = "mosh_common_workq",
	};

#if defined(CONFIG_SHELL_BACKEND_SERIAL)
	mosh_shell = shell_backend_uart_get_ptr();
#else
	mosh_shell = shell_backend_rtt_get_ptr();
#endif

	__ASSERT(mosh_shell != NULL, "Failed to get shell backend");

	/* Reset reason can only be read once, because the register needs to be cleared after
	 * reading. Memfault implementation reads the reset reason before modem_shell, so the
	 * register would always be empty.
	 */
	if (!IS_ENABLED(CONFIG_MEMFAULT)) {
		mosh_print_reset_reason();
	}

#if defined(CONFIG_NRF_CLOUD_REST) || defined(CONFIG_NRF_CLOUD_MQTT) || \
	defined(CONFIG_NRF_CLOUD_COAP)
#if defined(CONFIG_MOSH_IPERF3)
	/* Due to iperf3, we cannot let nrf cloud lib to initialize cJSON lib to be
	 * using kernel heap allocations (i.e. k_ prepending functions).
	 * Thus, we use standard c alloc/free, i.e. "system heap".
	 */
	struct nrf_cloud_os_mem_hooks hooks = {
		.malloc_fn = malloc,
		.calloc_fn = calloc,
		.free_fn = free,
	};
#else
	/* else: we default to Kernel heap */
	struct nrf_cloud_os_mem_hooks hooks = {
		.malloc_fn = k_malloc,
		.calloc_fn = k_calloc,
		.free_fn = k_free,
	};
#endif

	nrf_cloud_os_mem_hooks_init(&hooks);
#endif

	k_work_queue_start(
		&mosh_common_work_q,
		mosh_common_workq_stack,
		K_THREAD_STACK_SIZEOF(mosh_common_workq_stack),
		MOSH_COMMON_WORKQ_PRIORITY,
		&cfg);

	err = nrf_modem_lib_init();
	if (err) {
		/* Modem library initialization failed. */
		printk("Could not initialize nrf_modem_lib, err %d\n", err);
		printk("Fatal error\n");
		return 0;
	}

#if defined(CONFIG_MOSH_PPP)
	ppp_ctrl_init();
#endif
#if defined(CONFIG_MOSH_WORKER_THREADS)
	th_ctrl_init();
#endif
#if defined(CONFIG_MOSH_FOTA)
	err = fota_init();
	if (err) {
		printk("Could not initialize FOTA: %d\n", err);
	}
#endif

#if defined(CONFIG_MOSH_LOCATION)
	/* Location library should be initialized before LTE normal mode */
	location_ctrl_init();
#endif
	link_init();

	err = modem_info_init();
	if (err) {
		printk("Modem info could not be initialized: %d\n", err);
		return 0;
	}

	/* Version information printing uses the Modem information library, so it can not be done
	 * before the library has been initialized.
	 */
	mosh_print_version_info();

#if defined(CONFIG_DK_LIBRARY)
	err = dk_buttons_init(button_handler);
	if (err) {
		printk("Failed to initialize DK buttons library, error: %d\n", err);
	}
	err = dk_leds_init();
	if (err) {
		printk("Cannot initialize LEDs (err: %d)\n", err);
	}
#endif
	/* Application started successfully, mark image as OK to prevent
	 * revert at next reboot. If LwM2M Carrier library is enabled, allow
	 * the library to do it.
	 */
#if (defined(CONFIG_BOOTLOADER_MCUBOOT) && !defined(CONFIG_LWM2M_CARRIER))
	boot_write_img_confirmed();
#endif
	k_poll_signal_init(&mosh_signal);

#if defined(CONFIG_SHELL_BACKEND_SERIAL)
	/* Resize terminal width and height of the shell to have proper command editing. */
	shell_execute_cmd(mosh_shell, "resize");
#endif

#if defined(CONFIG_MOSH_STARTUP_CMDS)
	startup_cmd_ctrl_init();
#endif

#if defined(CONFIG_MOSH_AT_CMD_MODE)
	at_cmd_mode_sett_init();
	if (at_cmd_mode_sett_is_autostart_enabled()) {
		/* Start directly in AT cmd mode */
		at_cmd_mode_start(mosh_shell);
	}
#endif

	return 0;
}
