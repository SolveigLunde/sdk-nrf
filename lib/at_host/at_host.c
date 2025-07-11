/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <ctype.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/init.h>

#include <nrf_modem_at.h>
#include <modem/at_monitor.h>

LOG_MODULE_REGISTER(at_host, CONFIG_AT_HOST_LOG_LEVEL);

/* Stack definition for AT host workqueue */
K_THREAD_STACK_DEFINE(at_host_stack_area, CONFIG_AT_HOST_STACK_SIZE);

#define AT_BUF_SIZE CONFIG_AT_HOST_CMD_MAX_LEN

AT_MONITOR(at_host, ANY, notification_handler);

/** @brief Termination Modes. */
enum term_modes {
	MODE_NULL_TERM, /**< Null Termination */
	MODE_CR,        /**< CR Termination */
	MODE_LF,        /**< LF Termination */
	MODE_CR_LF,     /**< CR+LF Termination */
	MODE_COUNT      /* Counter of term_modes */
};

#define AT_HOST_UART_DEV_GET()                                                                     \
	DEVICE_DT_GET(COND_CODE_1(DT_HAS_CHOSEN(ncs_at_host_uart),                                 \
		(DT_CHOSEN(ncs_at_host_uart)), (DT_NODELABEL(uart0))))

#define IS_LOG_BACKEND_UART(_uart_dev)                                                             \
	(IS_ENABLED(CONFIG_LOG_BACKEND_UART) && COND_CODE_1(DT_HAS_CHOSEN(zephyr_log_uart),        \
		(_uart_dev == DEVICE_DT_GET(DT_CHOSEN(zephyr_log_uart))),                          \
		(COND_CODE_1(DT_HAS_CHOSEN(zephyr_console),                                        \
			(_uart_dev == DEVICE_DT_GET(DT_CHOSEN(zephyr_console))), (false)))))

static enum term_modes term_mode;
static const struct device *const uart_dev = AT_HOST_UART_DEV_GET();
static bool at_buf_busy; /* Guards at_buf while processing a command */
static char at_buf[AT_BUF_SIZE]; /* AT command and modem response buffer */
static struct k_work_q at_host_work_q;
static struct k_work cmd_send_work;

static inline void write_uart_string(const char *str)
{
	if (IS_LOG_BACKEND_UART(uart_dev)) {
		/* The chosen AT host UART device is also the UART log backend device.
		 * Therefore, log the AT response instead of writing directly to the UART device.
		 */
		LOG_RAW("%s", str);
		return;
	}

	/* Make sure there are both CR and LF between the command and the output (response or
	 * notification). Otherwise the output will be printed on top of the command or the
	 * output will have offset. 3GPP TS 27.007 specifies, that responses should be prefixed
	 * by CRLF. However, the string from the modem does not contain the CRLF prefix.
	 */
#if defined(CONFIG_LF_TERMINATION)
	uart_poll_out(uart_dev, '\r');
#endif
#if defined(CONFIG_CR_TERMINATION)
	uart_poll_out(uart_dev, '\n');
#endif

	/* Send characters until, but not including, null */
	for (size_t i = 0; str[i]; i++) {
		uart_poll_out(uart_dev, str[i]);
	}
}

static void notification_handler(const char *notification)
{
	/* Forward the data over UART */
	write_uart_string(notification);
}

static void cmd_send(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	/* Sending through string format rather than raw buffer in case
	 * the buffer contains characters that need to be escaped
	 */
	err = nrf_modem_at_cmd(at_buf, sizeof(at_buf), "%s", at_buf);
	if (err < 0) {
		LOG_ERR("Error while processing AT command: %d", err);
	}

	write_uart_string(at_buf);

	at_buf_busy = false;
	uart_irq_rx_enable(uart_dev);
}

static void uart_rx_handler(uint8_t character)
{
	static bool inside_quotes;
	static size_t at_cmd_len;

	/* Handle control characters */
	switch (character) {
	/* Backspace and DEL character */
	case 0x08:
	case 0x7F:
		if (at_cmd_len == 0) {
			return;
		}

		at_cmd_len--;
		/* If the removed character was a quote, need to toggle the flag. */
		if (at_buf[at_cmd_len] == '"') {
			inside_quotes = !inside_quotes;
		}
		return;
	}

	/* Handle termination characters, if outside quotes. */
	if (!inside_quotes) {
		switch (character) {
		case '\0':
			if (term_mode == MODE_NULL_TERM) {
				goto send;
			}
			LOG_WRN("Ignored null; would terminate string early.");
			return;
		case '\r':
			if (term_mode == MODE_CR) {
				goto send;
			}
			break;
		case '\n':
			if (term_mode == MODE_LF) {
				goto send;
			}
			if (term_mode == MODE_CR_LF &&
			    at_cmd_len > 0 &&
			    at_buf[at_cmd_len - 1] == '\r') {
				goto send;
			}
			break;
		}
	}

	/* Detect AT command buffer overflow, leaving space for null */
	if (at_cmd_len + 1 > sizeof(at_buf) - 1) {
		LOG_ERR("Buffer overflow, dropping '%c'\n", character);
		return;
	}

	/* Write character to AT buffer */
	at_buf[at_cmd_len] = character;
	at_cmd_len++;

	/* Handle special written character */
	if (character == '"') {
		inside_quotes = !inside_quotes;
	}

	return;
send:
	/* Terminate the command string */
	at_buf[at_cmd_len] = '\0';

	/* Reset UART handler state */
	inside_quotes = false;
	at_cmd_len = 0;

	/* Check for the presence of one printable non-whitespace character */
	for (const char *c = at_buf;; c++) {
		if (*c > ' ') {
			break;
		} else if (*c == '\0') {
			/* Drop command, if it has no such character */
			return;
		}
	}

	/* Send the command, if there is one to send */
	if (at_buf[0]) {
		/* Stop UART to protect at_buf */
		uart_irq_rx_disable(uart_dev);
		at_buf_busy = true;
		k_work_submit_to_queue(&at_host_work_q, &cmd_send_work);
	}
}

static void isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t character;

	uart_irq_update(dev);

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	/*
	 * Check that we are not sending data (buffer must be preserved then),
	 * and that a new character is available before handling each character
	 */
	while ((!at_buf_busy) &&
	       (uart_fifo_read(dev, &character, 1))) {
		uart_rx_handler(character);
	}
}

static int at_uart_init(const struct device *uart_dev)
{
	int err;
	uint8_t dummy;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	uint32_t start_time = k_uptime_get_32();

	/* Wait for the UART line to become valid */
	do {
		err = uart_err_check(uart_dev);
		if (err) {
			if (k_uptime_get_32() - start_time >
			    CONFIG_AT_HOST_UART_INIT_TIMEOUT) {
				LOG_ERR("UART check failed: %d. "
					"UART initialization timed out.", err);
				return -EIO;
			}

			LOG_INF("UART check failed: %d. "
				"Dropping buffer and retrying.", err);

			while (uart_fifo_read(uart_dev, &dummy, 1)) {
				/* Do nothing with the data */
			}
			k_sleep(K_MSEC(10));
		}
	} while (err);

	uart_irq_callback_set(uart_dev, isr);
	return err;
}

static int at_host_init(void)
{
	int err;
	enum term_modes mode = CONFIG_AT_HOST_TERMINATION;


	/* Choosing the termination mode */
	if (mode < MODE_COUNT) {
		term_mode = mode;
	} else {
		return -EINVAL;
	}

	/* Initialize the UART module */
	err = at_uart_init(uart_dev);
	if (err) {
		LOG_ERR("UART could not be initialized: %d", err);
		return -EFAULT;
	}

	k_work_init(&cmd_send_work, cmd_send);
	k_work_queue_start(&at_host_work_q, at_host_stack_area,
			   K_THREAD_STACK_SIZEOF(at_host_stack_area),
			   CONFIG_AT_HOST_THREAD_PRIO, NULL);
	uart_irq_rx_enable(uart_dev);

	return err;
}

SYS_INIT(at_host_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
