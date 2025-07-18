/*
 * Copyright (c) 2022 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <psa/crypto.h>
#include <stdint.h>

#include "tfm_sp_log.h"

#include "nrfx_gpiote.h"
#include "hal/nrf_egu.h"
#include "hal/nrf_timer.h"
#include "hal/nrf_spim.h"

#include "psa/service.h"
#include "psa_manifest/tfm_secure_peripheral_partition.h"

#include "util.h"

#include <zephyr/autoconf.h>
#include <zephyr/devicetree.h>
#include <soc_nrf_common.h>

#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>

#define BUTTON_PIN	NRF_DT_GPIOS_TO_PSEL(DT_NODELABEL(button0), gpios)

#if defined(CONFIG_SOC_SERIES_NRF53X)
#define SCK_PIN			NRF_PIN_PORT_TO_PIN_NUMBER(15, 1) /* P1.15 */
#define MOSI_PIN		NRF_PIN_PORT_TO_PIN_NUMBER(13, 1) /* P1.13 */
#define NRF_SPIM_N		NRF_SPIM3
#define NRF_TIMER_N		NRF_TIMER1
#define NRF_EGU_N_NS		NRF_EGU0_NS
#define NRF_GPIOTE_N		NRF_GPIOTE0
#define TFM_SPIM_N_IRQ_SIGNAL	TFM_SPIM3_IRQ_SIGNAL
#define TFM_TIMER_N_IRQ_SIGNAL	TFM_TIMER1_IRQ_SIGNAL
#define TFM_GPIOTE_N_IRQ_SIGNAL	TFM_GPIOTE0_IRQ_SIGNAL

#elif defined(CONFIG_SOC_SERIES_NRF91X)
#define SCK_PIN			NRF_PIN_PORT_TO_PIN_NUMBER(13, 0) /* P0.13 */
#define MOSI_PIN		NRF_PIN_PORT_TO_PIN_NUMBER(11, 0) /* P0.11 */
#define NRF_SPIM_N		NRF_SPIM3
#define NRF_TIMER_N		NRF_TIMER1
#define NRF_EGU_N_NS		NRF_EGU0_NS
#define NRF_GPIOTE_N		NRF_GPIOTE0
#define TFM_SPIM_N_IRQ_SIGNAL	TFM_SPIM3_IRQ_SIGNAL
#define TFM_TIMER_N_IRQ_SIGNAL	TFM_TIMER1_IRQ_SIGNAL
#define TFM_GPIOTE_N_IRQ_SIGNAL	TFM_GPIOTE0_IRQ_SIGNAL

#elif defined(CONFIG_SOC_SERIES_NRF54LX)
#define SCK_PIN			NRF_PIN_PORT_TO_PIN_NUMBER(12, 1) /* P1.12 */
#define MOSI_PIN		NRF_PIN_PORT_TO_PIN_NUMBER(11, 1) /* P1.11 */
#define NRF_SPIM_N		NRF_SPIM22
#define NRF_TIMER_N		NRF_TIMER10
#define NRF_EGU_N_NS		NRF_EGU10_NS
#define NRF_GPIOTE_N		NRF_GPIOTE20
#define TFM_SPIM_N_IRQ_SIGNAL	TFM_SPIM22_IRQ_SIGNAL
#define TFM_TIMER_N_IRQ_SIGNAL	TFM_TIMER10_IRQ_SIGNAL
#define TFM_GPIOTE_N_IRQ_SIGNAL	TFM_GPIOTE20_1_IRQ_SIGNAL

#endif

#define NRF_GPIOTE_CHANNEL 1

#define TIMER_RELOAD_VALUE (1*1000 * 1000)
static uint32_t m_button_count;
static uint32_t m_timer_count;
static uint32_t m_trigger_count;

static void timer_init(NRF_TIMER_Type *timer, uint32_t ticks)
{
	nrf_timer_mode_set(timer, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(timer, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(timer, NRF_TIMER_FREQ_1MHz);
	nrf_timer_cc_set(timer, NRF_TIMER_CC_CHANNEL0, ticks);
	/* Clear the timer once event is generated. */
	nrf_timer_shorts_enable(timer, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
}

static void timer_stop(NRF_TIMER_Type *timer)
{
	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_STOP);
	nrf_timer_int_disable(timer, NRF_TIMER_INT_COMPARE0_MASK);
	nrf_timer_event_clear(timer, NRF_TIMER_EVENT_COMPARE0);
}

static void timer_start(NRF_TIMER_Type *timer)
{
	timer_stop(timer);

	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_CLEAR);
	nrf_timer_int_enable(timer, NRF_TIMER_INT_COMPARE0_MASK);

	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_START);
}

static void timer_event_clear(NRF_TIMER_Type *timer)
{
	nrf_timer_event_clear(timer, NRF_TIMER_EVENT_COMPARE0);
}


#if defined(CONFIG_SOC_SERIES_NRF54LX)
psa_flih_result_t tfm_timer10_irq_flih(void)
#else
psa_flih_result_t tfm_timer1_irq_flih(void)
#endif
{
	timer_event_clear(NRF_TIMER_N);

	m_trigger_count++;

	if (m_trigger_count == 10) {
		m_trigger_count = 0;
		nrf_egu_task_trigger(NRF_EGU_N_NS, NRF_EGU_TASK_TRIGGER0);
		return PSA_FLIH_SIGNAL;
	}

	return PSA_FLIH_NO_SIGNAL;
}

static void gpio_init(uint32_t pin)
{
	nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpiote_event_configure(NRF_GPIOTE_N, NRF_GPIOTE_CHANNEL, pin,
								GPIOTE_CONFIG_POLARITY_HiToLo);

	nrf_gpiote_event_enable(NRF_GPIOTE_N, NRF_GPIOTE_CHANNEL);

#if defined(CONFIG_SOC_SERIES_NRF54LX)
	nrf_gpiote_int_group_enable(NRF_GPIOTE_N, 0x1,
				    GPIOTE_INTENSET1_PORT0SECURE_Msk | (1 << NRF_GPIOTE_CHANNEL));
#else
	nrf_gpiote_int_enable(NRF_GPIOTE_N, NRFX_BIT(NRF_GPIOTE_CHANNEL));
#endif
}

#if defined(CONFIG_SOC_SERIES_NRF54LX)
psa_flih_result_t tfm_gpiote20_1_irq_flih(void)
#else
psa_flih_result_t tfm_gpiote0_irq_flih(void)
#endif
{
	nrf_gpiote_event_clear(NRF_GPIOTE_N, nrf_gpiote_in_event_get(NRF_GPIOTE_CHANNEL));

	nrf_egu_task_trigger(NRF_EGU_N_NS, NRF_EGU_TASK_TRIGGER0);
	return PSA_FLIH_SIGNAL;
}

static void spim_init(uint32_t sck_pin, uint32_t mosi_pin)
{
	nrf_spim_pins_set(NRF_SPIM_N, sck_pin, mosi_pin, NRF_SPIM_PIN_NOT_CONNECTED);
	nrf_spim_configure(NRF_SPIM_N, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
#if SPIM0_FEATURE_HARDWARE_CSN_PRESENT
	nrf_spim_csn_configure(NRF_SPIM_N,
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_CSN_POL_LOW,
		0);
#endif

#if NRF_SPIM_HAS_FREQUENCY
	nrf_spim_frequency_set(NRF_SPIM_N, NRF_SPIM_FREQ_2M);
#endif

	nrf_spim_int_enable(NRF_SPIM_N, NRF_SPIM_INT_ENDTX_MASK);
	nrf_spim_enable(NRF_SPIM_N);
}

static void spim_send(const uint8_t *buf, size_t len)
{
	nrf_spim_tx_buffer_set(NRF_SPIM_N, buf, len);
	nrf_spim_rx_buffer_set(NRF_SPIM_N, NULL, 0);

	nrf_spim_task_trigger(NRF_SPIM_N, NRF_SPIM_TASK_START);
}

static void spim_event_clear(void)
{
	nrf_spim_event_clear(NRF_SPIM_N, NRF_SPIM_EVENT_ENDTX);
}

static size_t generate_msg(uint8_t *msg_buf, uint8_t msg_len,
			   uint32_t timer_count, uint32_t button_count)
{
	size_t hash_length;
	uint8_t secret[4 + 4];
	uint8_t hash[32];

	util_put_le32(timer_count, &secret[0]);
	util_put_le32(button_count, &secret[4]);

	psa_status_t status = psa_hash_compute(PSA_ALG_SHA_256,
				secret, sizeof(secret),
				hash, sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		psa_panic();
	}

	return util_bin2hex(hash, hash_length, msg_buf, msg_len);
}

static void send_msg(void)
{
	static uint8_t msg_buf[2 * 32 + 1];
	size_t msg_len;

	msg_len = generate_msg(msg_buf, sizeof(msg_buf),
			       m_timer_count, m_button_count);
	psa_irq_enable(TFM_SPIM_N_IRQ_SIGNAL);

	spim_send(msg_buf, msg_len);
	if (psa_wait(TFM_SPIM_N_IRQ_SIGNAL, PSA_BLOCK) != TFM_SPIM_N_IRQ_SIGNAL) {
		psa_panic();
	}

	spim_event_clear();
	psa_eoi(TFM_SPIM_N_IRQ_SIGNAL);

	psa_irq_disable(TFM_SPIM_N_IRQ_SIGNAL);
}

static void spp_signals_process(psa_signal_t signals)
{
	if (signals & TFM_TIMER_N_IRQ_SIGNAL) {
		m_timer_count++;

		LOG_INFFMT("IRQ: TIMER count: %d\r\n", m_timer_count);

		psa_reset_signal(TFM_TIMER_N_IRQ_SIGNAL);
	}
	if (signals & TFM_GPIOTE_N_IRQ_SIGNAL) {
		m_button_count++;

		LOG_INFFMT("IRQ: GPIOTE0 count: %d\r\n", m_button_count);

		psa_reset_signal(TFM_GPIOTE_N_IRQ_SIGNAL);
	}
}

static void spp_send(void)
{
	psa_status_t status;
	psa_msg_t msg;

	status = psa_get(TFM_SPP_SEND_SIGNAL, &msg);

	send_msg();

	psa_reply(msg.handle, status);
}

static void spp_signal_handle(psa_signal_t signals)
{
	psa_status_t status;
	psa_msg_t msg;

	status = psa_get(TFM_SPP_PROCESS_SIGNAL, &msg);

	spp_signals_process(signals);

	psa_reply(msg.handle, status);
}

static void spp_init(void)
{
	timer_init(NRF_TIMER_N, TIMER_RELOAD_VALUE);
	timer_start(NRF_TIMER_N);

	psa_irq_enable(TFM_TIMER_N_IRQ_SIGNAL);

	gpio_init(BUTTON_PIN);

	psa_irq_enable(TFM_GPIOTE_N_IRQ_SIGNAL);

	spim_init(SCK_PIN, MOSI_PIN);
}

psa_status_t tfm_spp_main(void)
{
	psa_signal_t signals = 0;

	spp_init();

	while (1) {
		signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);
		if (signals & TFM_SPP_SEND_SIGNAL) {
			spp_send();
		} else if (signals & TFM_SPP_PROCESS_SIGNAL) {
			spp_signal_handle(signals);
		} else if (signals & (TFM_TIMER_N_IRQ_SIGNAL |
				      TFM_GPIOTE_N_IRQ_SIGNAL)) {
			/* Partition schedule was run while signals was set. */
			spp_signals_process(signals);
		} else {
			psa_panic();
		}
	}

	return PSA_ERROR_SERVICE_FAILURE;
}
