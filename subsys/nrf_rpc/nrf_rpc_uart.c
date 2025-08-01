/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrf_rpc.h>
#include <nrf_rpc_tr.h>
#include <nrf_rpc/nrf_rpc_uart.h>
#include <nrf_rpc_errno.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

LOG_MODULE_REGISTER(nrf_rpc_uart, CONFIG_NRF_RPC_TR_LOG_LEVEL);

#define CRC_SIZE sizeof(uint16_t)

enum {
	HDLC_CHAR_ESCAPE = 0x7d,
	HDLC_CHAR_DELIMITER = 0x7e,
};

enum flip_state {
	FLIP_ZERO,
	FLIP_ONE
};

struct trx_flips {
	uint8_t tx_flip : 1;
	uint8_t rx_flip_any : 1;
	uint16_t last_rx_crc;
};

enum hdlc_state {
	/* Ignore incoming bytes until the delimiter is found. */
	HDLC_STATE_UNSYNC,
	/* Append incoming bytes to the output buffer. */
	HDLC_STATE_FRAME,
	/* Found the delimeter when the output buffer was non-empty. */
	HDLC_STATE_FRAME_FOUND,
	/* Found the escape byte. Append the following byte XORed with 0x20 to the output buffer. */
	HDLC_STATE_ESCAPE,
};

struct hdlc_decode_ctx {
	enum hdlc_state state;
	/* The number of bytes of the current packet that have been decoded so far. */
	uint16_t len;
	/* The capacity of the buffer to store a decoded packet. */
	uint16_t capacity;
};

struct nrf_rpc_uart {
	const struct device *uart;
	nrf_rpc_tr_receive_handler_t receive_callback;
	void *receive_ctx;
	const struct nrf_rpc_tr *transport;

	/* RX ring buffer populated by UART ISR */
	uint8_t rx_buffer[CONFIG_NRF_RPC_UART_RX_RINGBUF_SIZE];
	struct ring_buf rx_ringbuf;

	/* RX work to consume and decode bytes from RX ring buffer */
	struct k_work rx_work;
	struct k_work_q rx_workq;

	K_KERNEL_STACK_MEMBER(rx_workq_stack, CONFIG_NRF_RPC_UART_RX_THREAD_STACK_SIZE);

	/* HDLC ack decoding state */
	struct hdlc_decode_ctx rx_ack_ctx;
	uint8_t rx_ack[CRC_SIZE];

	/* HDLC packet decoding state */
	struct hdlc_decode_ctx rx_pkt_ctx;
	uint8_t rx_pkt[CONFIG_NRF_RPC_UART_MAX_PACKET_SIZE];

	/* Ack waiting semaphore */
	struct k_sem ack_sem;
	uint16_t ack_payload;
	struct k_mutex ack_tx_lock;
	struct trx_flips flips;

	/* TX lock */
	struct k_mutex tx_lock;
};

static void log_hexdump_dbg(const uint8_t *data, size_t length, const char *fmt, ...)
{
	if (IS_ENABLED(CONFIG_NRF_RPC_TR_LOG_LEVEL_DBG)) {
		va_list ap;
		char message[32];

		va_start(ap, fmt);
		vsnprintk(message, sizeof(message), fmt, ap);
		va_end(ap);

		LOG_HEXDUMP_DBG(data, length, message);
	}
}

static void send_byte(const struct device *dev, uint8_t byte);

static void ack_rx(struct nrf_rpc_uart *uart_tr)
{
	if (!IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE) || uart_tr->rx_ack_ctx.len != CRC_SIZE) {
		log_hexdump_dbg(uart_tr->rx_ack, uart_tr->rx_ack_ctx.len, ">>> RX invalid frame");
		return;
	}

	uint16_t rx_ack = sys_get_le16(uart_tr->rx_ack);

	LOG_DBG(">>> RX ack %04x", rx_ack);

	if (uart_tr->ack_payload != rx_ack) {
		LOG_WRN("Received ack %04x but expected %04x", rx_ack, uart_tr->ack_payload);
		return;
	}

	k_sem_give(&uart_tr->ack_sem);
}

static void ack_tx(struct nrf_rpc_uart *uart_tr, uint16_t ack_pld)
{
	uint8_t ack[2];

	if (!IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE)) {
		return;
	}

	sys_put_le16(ack_pld, ack);
	k_mutex_lock(&uart_tr->ack_tx_lock, K_FOREVER);
	LOG_DBG("<<< TX ack %04x", ack_pld);

	uart_poll_out(uart_tr->uart, HDLC_CHAR_DELIMITER);

	send_byte(uart_tr->uart, ack[0]);
	send_byte(uart_tr->uart, ack[1]);

	uart_poll_out(uart_tr->uart, HDLC_CHAR_DELIMITER);

	k_mutex_unlock(&uart_tr->ack_tx_lock);
}

static uint16_t tx_flip(struct nrf_rpc_uart *uart_tr, uint16_t crc_val)
{
	if (!IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE)) {
		return crc_val;
	}

	if (uart_tr->flips.tx_flip == FLIP_ZERO) {
		crc_val &= 0x7fffu;
		uart_tr->flips.tx_flip = FLIP_ONE;
	} else {
		crc_val |= 0x8000u;
		uart_tr->flips.tx_flip = FLIP_ZERO;
	}

	return crc_val;
}

static bool rx_flip_check(struct nrf_rpc_uart *uart_tr, uint16_t crc_val)
{
	uint16_t last_rx_crc;

	if (!IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE)) {
		return false;
	}

	last_rx_crc = uart_tr->flips.last_rx_crc;
	uart_tr->flips.last_rx_crc = crc_val;

	if (uart_tr->flips.rx_flip_any == 1 || last_rx_crc != crc_val) {
		uart_tr->flips.rx_flip_any = 0;
		return false;
	}

	return true;
}

static bool crc_compare(uint16_t rx_crc, uint16_t calc_crc)
{
	if (IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE)) {
		return (rx_crc & 0x7fffu) == (calc_crc & 0x7fffu);
	}

	return rx_crc == calc_crc;
}

static void hdlc_decode_byte(struct hdlc_decode_ctx *ctx, uint8_t *out, uint8_t in)
{
	switch (ctx->state) {
	case HDLC_STATE_UNSYNC:
		if (in == HDLC_CHAR_DELIMITER) {
			ctx->len = 0;
			ctx->state = HDLC_STATE_FRAME;
		}
		return;
	case HDLC_STATE_FRAME_FOUND:
		ctx->len = 0;
		ctx->state = HDLC_STATE_FRAME;
		__fallthrough;
	case HDLC_STATE_FRAME:
		if (in == HDLC_CHAR_DELIMITER) {
			if (ctx->len > 0) {
				ctx->state = HDLC_STATE_FRAME_FOUND;
			}
			return;
		} else if (in == HDLC_CHAR_ESCAPE) {
			ctx->state = HDLC_STATE_ESCAPE;
			return;
		}
		break;
	case HDLC_STATE_ESCAPE:
		in ^= 0x20;
		ctx->state = HDLC_STATE_FRAME;
		break;
	}

	if (ctx->len >= ctx->capacity) {
		/* Ignore too long frame */
		ctx->state = HDLC_STATE_UNSYNC;
		return;
	}

	out[ctx->len++] = in;
}

static void work_handler(struct k_work *work)
{
	struct nrf_rpc_uart *uart_tr = CONTAINER_OF(work, struct nrf_rpc_uart, rx_work);
	uint8_t *data;
	size_t len;
	int ret;
	uint16_t crc_received = 0;
	uint16_t crc_calculated = 0;

	while (!ring_buf_is_empty(&uart_tr->rx_ringbuf)) {
		len = ring_buf_get_claim(&uart_tr->rx_ringbuf, &data,
					 CONFIG_NRF_RPC_UART_MAX_PACKET_SIZE);
		for (size_t i = 0; i < len; i++) {
			hdlc_decode_byte(&uart_tr->rx_pkt_ctx, uart_tr->rx_pkt, data[i]);

			if (uart_tr->rx_pkt_ctx.state != HDLC_STATE_FRAME_FOUND) {
				continue;
			}

			/* ACKs are already handled in ISR, so process only normal packets here */
			if (uart_tr->rx_pkt_ctx.len <= CRC_SIZE) {
				continue;
			}

			uart_tr->rx_pkt_ctx.len -= CRC_SIZE;
			crc_received = sys_get_le16(uart_tr->rx_pkt + uart_tr->rx_pkt_ctx.len);
			crc_calculated =
				crc16_ccitt(0xffff, uart_tr->rx_pkt, uart_tr->rx_pkt_ctx.len);

			log_hexdump_dbg(uart_tr->rx_pkt, uart_tr->rx_pkt_ctx.len,
					">>> RX packet %04x", crc_received);

			if (!crc_compare(crc_received, crc_calculated)) {
				LOG_ERR("Invalid packet CRC: calculated %04x but received %04x",
					crc_calculated, crc_received);
				continue;
			}

			ack_tx(uart_tr, crc_received);

			if (rx_flip_check(uart_tr, crc_received)) {
				LOG_WRN("Duplicate packet %04x", crc_received);
			} else {
				uart_tr->receive_callback(uart_tr->transport, uart_tr->rx_pkt,
							  uart_tr->rx_pkt_ctx.len,
							  uart_tr->receive_ctx);
			}
		}

		ret = ring_buf_get_finish(&uart_tr->rx_ringbuf, len);
		if (ret < 0) {
			LOG_DBG("Cannot flush ring buffer: %d", ret);
		}
	}
}

static void decode_ack(struct nrf_rpc_uart *inst, const uint8_t *in, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		hdlc_decode_byte(&inst->rx_ack_ctx, inst->rx_ack, in[i]);

		if (inst->rx_ack_ctx.state == HDLC_STATE_FRAME_FOUND) {
			ack_rx(inst);
		}
	}
}

static void serial_cb(const struct device *uart, void *user_data)
{
	struct nrf_rpc_uart *uart_tr = user_data;
	uint32_t rx_len;
	uint8_t *rx_buffer;
	bool new_data = false;

	while (uart_irq_update(uart) && uart_irq_rx_ready(uart)) {
		rx_len = ring_buf_put_claim(&uart_tr->rx_ringbuf, &rx_buffer,
					    uart_tr->rx_ringbuf.size);
		if (rx_len > 0) {
			rx_len = uart_fifo_read(uart, rx_buffer, rx_len);
			decode_ack(uart_tr, rx_buffer, rx_len);
			int err = ring_buf_put_finish(&uart_tr->rx_ringbuf, rx_len);
			(void)err; /*silence the compiler*/
			__ASSERT_NO_MSG(err == 0);
			if (rx_len <= 0) {
				continue;
			} else {
				new_data = true;
			}
		} else {
			uint8_t dummy;

			/* No space in the ring buffer - consume byte. */
			LOG_WRN("RX ring buffer full");

			rx_len = uart_fifo_read(uart, &dummy, 1);
		}
	}

	if (new_data) {
		k_work_submit_to_queue(&uart_tr->rx_workq, &uart_tr->rx_work);
	}
}

static int init(const struct nrf_rpc_tr *transport, nrf_rpc_tr_receive_handler_t receive_cb,
		void *context)
{
	struct nrf_rpc_uart *uart_tr = transport->ctx;

	if (uart_tr->transport != NULL) {
		return 0;
	}

	uart_tr->transport = transport;

	LOG_DBG("init called");

	if (receive_cb == NULL) {
		return -NRF_EINVAL;
	}
	uart_tr->receive_callback = receive_cb;
	uart_tr->receive_ctx = context;

	if (!device_is_ready(uart_tr->uart)) {
		LOG_ERR("UART device not found!");
		return -NRF_ENOENT;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_tr->uart, serial_cb, uart_tr);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API\n");
		} else {
			LOG_ERR("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}

	k_mutex_init(&uart_tr->tx_lock);

	if (IS_ENABLED(CONFIG_NRF_RPC_UART_RELIABLE)) {
		k_mutex_init(&uart_tr->ack_tx_lock);
		k_sem_init(&uart_tr->ack_sem, 0, 1);
		uart_tr->flips.tx_flip = FLIP_ZERO;
		uart_tr->flips.rx_flip_any = 1;
	}

	k_work_queue_init(&uart_tr->rx_workq);
	k_work_queue_start(&uart_tr->rx_workq, uart_tr->rx_workq_stack,
			   K_THREAD_STACK_SIZEOF(uart_tr->rx_workq_stack), K_PRIO_PREEMPT(0), NULL);

	k_work_init(&uart_tr->rx_work, work_handler);
	ring_buf_init(&uart_tr->rx_ringbuf, sizeof(uart_tr->rx_buffer), uart_tr->rx_buffer);

	uart_tr->rx_pkt_ctx.state = HDLC_STATE_UNSYNC;
	uart_tr->rx_pkt_ctx.capacity = sizeof(uart_tr->rx_pkt);
	uart_tr->rx_ack_ctx.state = HDLC_STATE_UNSYNC;
	uart_tr->rx_ack_ctx.capacity = sizeof(uart_tr->rx_ack);
	uart_irq_rx_enable(uart_tr->uart);
	nrf_rpc_uart_initialized_hook(uart_tr->uart);

	return 0;
}

static void send_byte(const struct device *dev, uint8_t byte)
{
	if (byte == HDLC_CHAR_DELIMITER || byte == HDLC_CHAR_ESCAPE) {
		uart_poll_out(dev, HDLC_CHAR_ESCAPE);
		byte ^= 0x20;
	}

	uart_poll_out(dev, byte);
}

static int send(const struct nrf_rpc_tr *transport, const uint8_t *data, size_t length)
{
	uint8_t crc[2];
	uint16_t crc_val;
	bool acked = true;
	struct nrf_rpc_uart *uart_tr = transport->ctx;

	k_mutex_lock(&uart_tr->tx_lock, K_FOREVER);

	crc_val = crc16_ccitt(0xffff, data, length);
	crc_val = tx_flip(uart_tr, crc_val);
	log_hexdump_dbg(data, length, "<<< TX packet %04x", crc_val);

#if CONFIG_NRF_RPC_UART_RELIABLE
	int attempts = 0;

	uart_tr->ack_payload = crc_val;
	acked = false;

	do {
		attempts++;
		k_mutex_lock(&uart_tr->ack_tx_lock, K_FOREVER);
		k_sem_reset(&uart_tr->ack_sem);
#endif /* CONFIG_NRF_RPC_UART_RELIABLE */

		uart_poll_out(uart_tr->uart, HDLC_CHAR_DELIMITER);

		for (size_t i = 0; i < length; i++) {
			send_byte(uart_tr->uart, data[i]);
		}

		sys_put_le16(crc_val, crc);
		send_byte(uart_tr->uart, crc[0]);
		send_byte(uart_tr->uart, crc[1]);

		uart_poll_out(uart_tr->uart, HDLC_CHAR_DELIMITER);

#if CONFIG_NRF_RPC_UART_RELIABLE
		k_mutex_unlock(&uart_tr->ack_tx_lock);
		if (k_sem_take(&uart_tr->ack_sem, K_MSEC(CONFIG_NRF_RPC_UART_ACK_WAITING_TIME)) ==
		    0) {
			acked = true;
			LOG_DBG("Acked successfully");
		} else {
			LOG_WRN("Ack timeout");
		}
	} while (!acked && attempts < CONFIG_NRF_RPC_UART_TX_ATTEMPTS);
#endif /* CONFIG_NRF_RPC_UART_RELIABLE */

	k_free((void *)data);

	k_mutex_unlock(&uart_tr->tx_lock);

	return acked ? 0 : -EPROTO;
}

static void *tx_buf_alloc(const struct nrf_rpc_tr *transport, size_t *size)
{
	void *data = NULL;

	data = k_malloc(*size);
	if (!data) {
		LOG_ERR("Failed to allocate TX buffer");
		goto error;
	}

	return data;

error:
	/* It should fail to avoid writing to NULL buffer. */
	k_oops();
	*size = 0;
	return NULL;
}

static void tx_buf_free(const struct nrf_rpc_tr *transport, void *buf)
{
	ARG_UNUSED(transport);

	k_free(buf);
}

__weak void nrf_rpc_uart_initialized_hook(const struct device *uart_dev)
{
}

const struct nrf_rpc_tr_api nrf_rpc_uart_service_api = {
	.init = init,
	.send = send,
	.tx_buf_alloc = tx_buf_alloc,
	.tx_buf_free = tx_buf_free,
};

#define NRF_RPC_UART_INSTANCE(node_id) _CONCAT(nrf_rpc_inst_, DT_DEP_ORD(node_id))

#define NRF_RPC_UART_TRANSPORT_DEFINE(node_id)                                                     \
	struct nrf_rpc_uart NRF_RPC_UART_INSTANCE(node_id) = {                                     \
		.uart = DEVICE_DT_GET(node_id),                                                    \
		.receive_callback = NULL,                                                          \
		.transport = NULL,                                                                 \
	};                                                                                         \
	const struct nrf_rpc_tr NRF_RPC_UART_TRANSPORT(node_id) = {                                \
		.api = &nrf_rpc_uart_service_api,                                                  \
		.ctx = &NRF_RPC_UART_INSTANCE(node_id),                                            \
	};

DT_FOREACH_STATUS_OKAY(nordic_nrf_uarte, NRF_RPC_UART_TRANSPORT_DEFINE);
