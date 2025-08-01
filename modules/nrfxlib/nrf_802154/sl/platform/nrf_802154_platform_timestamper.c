/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "platform/nrf_802154_platform_timestamper.h"

#include <haly/nrfy_dppi.h>
#include <haly/nrfy_grtc.h>
#include <hal/nrf_dppi.h>
#include <hal/nrf_ppib.h>
#if defined(NRF54H_SERIES)
#include <hal/nrf_ipct.h>
#endif

#include <assert.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>

static int32_t m_timestamp_cc_channel;

#if defined(NRF54H_SERIES)
/* To trigger GRTC.TASKS_CAPTURE[#cc] with RADIO.EVENT_{?}, the following connection chain must be
 * created:
 *    - starting from LOCAL (RADIO core) domain:
 *        {a} RADIO.EVENT_{?}  --> DPPIC_020
 *        {b} DPPIC_020        --> IPCT_radio
 *    - entering the GLOBAL "Main power" domain (G1):
 *        {c} IPCT_radio       --> IPCT_130
 *        {d} IPCT_130         --> DPPIC_130
 *        {e} DPPIC_130        --> PPIB_130
 *        {f} PPIB_130         --> PPIB_133
 *    - ending in the GLOBAL "Active power" domain (G2):
 *        {g} PPIB_133         --> DPPIC_132
 *        {h} DPPIC_132        --> GRTC.CC
 */

/* Peripherals used for timestamping - located in local domain (_L_) */
/*   - DPPIC_L : DPPIC020 (RADIOCORE.DPPIC020) */
#define DPPIC_L_INST                 NRF_DPPIC020

/*   - IPCT_L : NRF_IPCT (RADIOCORE.IPCT) */
#define IPCT_L_TS_CHANNEL            2
#define IPCT_L_TASK_SEND             NRFX_CONCAT_2(NRF_IPCT_TASK_SEND_, IPCT_L_TS_CHANNEL)

/* Peripherals used for timestamping - located in global "Main power domain" (_G1_) */
/*   - IPCT_G1 : IPCT130_S */
#define IPCT_G1_INST                 NRF_IPCT130_S
#define IPCT_G1_TS_CHANNEL           2
#if defined(NRF54H20_ENGA_XXAA)
#define IPCT_G1_SHORTS               IPCT_SHORTS_RECEIVE2_ACK2_Msk
#else
#define IPCT_G1_SHORTS               IPCT_SHORTS_RECEIVE2_FLUSH2_Msk
#endif
#define IPCT_G1_EVENT_RECEIVE        NRFX_CONCAT_2(NRF_IPCT_EVENT_RECEIVE_, IPCT_G1_TS_CHANNEL)

/*   - DPPIC_G1 : DPPIC130_S */
/* The channel must be in the [0..7] range to satisfy the requirements for the dependent macros */
#define DPPIC_G1_INST                NRF_DPPIC130_S
#define DPPIC_G1_TS_CHANNEL          3

/*   - PPIB_G1 : PPIB130_S */
#define PPIB_G1_TS_CHANNEL           (DPPIC_G1_TS_CHANNEL + 8) /* hw-fixed dependency */

/* Peripherals used for timestamping - located in global "Active power domain" (_G2_) */
/*   - PPIB_G2 : PPIB133_S */
#define PPIB_G2_TS_CHANNEL           (PPIB_G1_TS_CHANNEL - 8) /* hw-fixed dependency */

/*   - DPPIC_G2 : DPPIC132_S */
#define DPPIC_G2_INST                NRF_DPPIC132_S
#define DPPIC_G2_TS_CHANNEL          (PPIB_G2_TS_CHANNEL + 0) /* hw-fixed dependency */

#if (PPIB_G1_TS_CHANNEL < 8) || (PPIB_G1_TS_CHANNEL > 15)
/* Only this range of channels can be connected to PPIB133 */
#error PPIB_G1_TS_CHANNEL is required to be in the [8..15] range
#endif

/* Ensure something similar to this is present in the DT:
 * &dppic130 {
 *	owned-channels = <3>;
 *	source-channels = <3>;
 * };
 * where 3 corresponds to DPPIC_G1_TS_CHANNEL.
 */
#define _TS_DT_CHECK_DPPIC_G1_CHANNEL(node_id, prop, idx) \
	(DT_PROP_BY_IDX(node_id, prop, idx) == DPPIC_G1_TS_CHANNEL) ||

#define TS_DT_HAS_RESERVED_DPPIC_G1_CHANNEL		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(dppic130), source_channels), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(dppic130), \
					  source_channels, \
					  _TS_DT_CHECK_DPPIC_G1_CHANNEL) false), \
		    (false))

BUILD_ASSERT(TS_DT_HAS_RESERVED_DPPIC_G1_CHANNEL,
	     "The required DPPIC_G1 channel is not reserved");

/* Ensure something similar to this is present in the DT:
 * &dppic132 {
 *	owned-channels = <3>;
 *	sink-channels = <3>;
 * };
 * where 3 corresponds to DPPIC_G2_TS_CHANNEL.
 */
#define _TS_DT_CHECK_DPPIC_G2_CHANNEL(node_id, prop, idx) \
	(DT_PROP_BY_IDX(node_id, prop, idx) == DPPIC_G2_TS_CHANNEL) ||

#define TS_DT_HAS_RESERVED_DPPIC_G2_CHANNEL		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(dppic132), sink_channels), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(dppic132), \
					  sink_channels, \
					  _TS_DT_CHECK_DPPIC_G2_CHANNEL) false), \
		    (false))

BUILD_ASSERT(TS_DT_HAS_RESERVED_DPPIC_G2_CHANNEL,
	     "The required DPPIC_G2 channel is not reserved");

/* Ensure something similar to this is present in the DT:
 * &cpurad_ipct {
 *	source-channel-links = <2 13 2>;
 * };
 * where first 2 corresponds to IPCT_L_TS_CHANNEL.
 */
#define _TS_DT_CHECK_IPCT_L_LINK(node_id, prop, idx) \
	((DT_PROP_BY_IDX(node_id, prop, idx) == IPCT_L_TS_CHANNEL) && ((idx) % 3 == 0)) ||


#define TS_DT_HAS_RESERVED_IPCT_L_LINK		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(cpurad_ipct), source_channel_links), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(cpurad_ipct), \
					  source_channel_links, \
					  _TS_DT_CHECK_IPCT_L_LINK) false), \
		    (false))

/* NOTE: this not verifying the allocation as the device tree property is an
 * array of triplets that is difficult to separate using macros.
 */
BUILD_ASSERT(TS_DT_HAS_RESERVED_IPCT_L_LINK,
	     "The required IPCT_radio link is not reserved");

/* Ensure something similar to this is present in the DT:
 * &ipct130 {
 *	sink-channel-links = <2 3 2>;
 * };
 * where first 2 corresponds to IPCT_G1_TS_CHANNEL.
 */
#define _TS_DT_CHECK_IPCT_G1_LINK(node_id, prop, idx) \
	((DT_PROP_BY_IDX(node_id, prop, idx) == IPCT_G1_TS_CHANNEL) && ((idx) % 3 == 0)) ||


#define TS_DT_HAS_RESERVED_IPCT_G1_LINK		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(ipct130), sink_channel_links), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(ipct130), \
					  sink_channel_links, \
					  _TS_DT_CHECK_IPCT_G1_LINK) false), \
		    (false))

/* NOTE: this not verifying the allocation as the device tree property is an
 * array of triplets that is difficult to separate using macros.
 */
BUILD_ASSERT(TS_DT_HAS_RESERVED_IPCT_G1_LINK,
	     "The required IPCT_G1 link is not reserved");


void nrf_802154_platform_timestamper_cross_domain_connections_setup(void)
{
	/* {c} IPCT_radio --> IPCT_130
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring the UICR->IPCMAP[] registers.
	 *
	 * Only enable auto confirmations on destination - IPCT_130.
	 */
	nrf_ipct_shorts_enable(IPCT_G1_INST, IPCT_G1_SHORTS);

	/* {d} IPCT_130 --> DPPIC_130 */
	nrf_ipct_publish_set(IPCT_G1_INST, IPCT_G1_EVENT_RECEIVE, DPPIC_G1_TS_CHANNEL);

	/* {e} DPPIC_130 --> PPIB_130
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring the UICR->DPPI.GLOBAL[].CH.LINK.SOURCE registers.
	 *
	 * Only enable relevant DPPIC_G1_INST channel.
	 */
	nrfy_dppi_channels_enable(DPPIC_G1_INST, 1UL << DPPIC_G1_TS_CHANNEL);

	/* {f} PPIB_130 --> PPIB_133
	 * One of HW-fixed connections (channels [8..15] --> [0..7]), so nothing to do.
	 */

	/* {g} PPIB_133 --> DPPIC_132
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring of the UICR->DPPI.GLOBAL[].CH.LINK.SINK registers.
	 */

	/* {h} DPPIC_132 --> GRTC.CC */
	nrf_grtc_task_t capture_task =
		nrfy_grtc_sys_counter_capture_task_get(m_timestamp_cc_channel);
	NRF_DPPI_ENDPOINT_SETUP(
		nrfy_grtc_task_address_get(NRF_GRTC, capture_task), DPPIC_G2_TS_CHANNEL);

	/* Enable relevant DPPIC_G2_INST channel. */
	nrfy_dppi_channels_enable(DPPIC_G2_INST, 1UL << DPPIC_G2_TS_CHANNEL);
}

void nrf_802154_platform_timestamper_local_domain_connections_setup(uint32_t dppi_ch)
{
	/* {a} RADIO.EVENT_{?} --> DPPIC_020[dppi_ch]
	 * It is the responsibility of the user of this platform to make the {a} connection
	 * and pass the DPPI channel number as a parameter here.
	 */

	/* {b} DPPIC_020[dppi_ch] to IPCT_radio. */
	nrf_ipct_subscribe_set(NRF_IPCT, IPCT_L_TASK_SEND, dppi_ch);
}

#elif defined(NRF54L_SERIES)

/* To trigger GRTC.TASKS_CAPTURE[#cc] with RADIO.EVENT_{?}, the following connection chain must be
 * created:
 *    - starting from RADIO domain (_R_):
 *        {a} RADIO.EVENT_{?}  --> DPPIC_10
 *        {b} DPPIC_10         --> PPIB_11
 *    - crossing domain boundaries
 *        {c} PPIB_11          --> PPIB_21
 *    - ending in the PERI domain (_P_):
 *        {d} PPIB_21          --> DPPIC_20
 *        {e} DPPIC_20         --> GRTC.CC
 */

#include <nrfx_ppib.h>
#include <nrfx_dppi.h>

#define INVALID_CHANNEL UINT8_MAX

static nrfx_dppi_t dppi20 = NRFX_DPPI_INSTANCE(20);
static nrfx_ppib_interconnect_t ppib11_21 = NRFX_PPIB_INTERCONNECT_INSTANCE(11, 21);
static uint8_t peri_dppi_ch = INVALID_CHANNEL;
static uint8_t peri_ppib_ch = INVALID_CHANNEL;

void nrf_802154_platform_timestamper_cross_domain_connections_setup(void)
{
	nrfx_err_t err;

	err = nrfx_dppi_channel_alloc(&dppi20, &peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppib_channel_alloc(&ppib11_21, &peri_ppib_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	/* {d} PPIB_21 --> DPPIC_20 */
	NRF_DPPI_ENDPOINT_SETUP(
		nrfx_ppib_receive_event_address_get(&ppib11_21.right, peri_ppib_ch), peri_dppi_ch);

	/* {e} DPPIC_20[dppi_ch] --> GRTC.CC[cc_channel] */
	nrf_grtc_task_t capture_task =
		nrfy_grtc_sys_counter_capture_task_get(m_timestamp_cc_channel);
	NRF_DPPI_ENDPOINT_SETUP(nrfy_grtc_task_address_get(NRF_GRTC, capture_task), peri_dppi_ch);

	err = nrfx_dppi_channel_enable(&dppi20, peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
}

void nrf_802154_platform_timestamper_local_domain_connections_setup(uint32_t dppi_ch)
{
	z_nrf_grtc_timer_capture_prepare(m_timestamp_cc_channel);

	/* {a} RADIO.EVENT_{?} --> DPPIC_10
	 * It is the responsibility of the user of this platform to make the {a} connection
	 * and pass the DPPI channel number as a parameter here.
	 */

	/* {b} DPPIC_10 --> PPIB_11 */
	NRF_DPPI_ENDPOINT_SETUP(
		nrfx_ppib_send_task_address_get(&ppib11_21.left, peri_ppib_ch), dppi_ch);
}

#endif

void nrf_802154_platform_timestamper_init(void)
{
	m_timestamp_cc_channel = z_nrf_grtc_timer_chan_alloc();
	assert(m_timestamp_cc_channel >= 0);
}

void nrf_802154_platform_timestamper_cross_domain_connections_clear(void)
{
	nrf_grtc_task_t capture_task =
		nrfy_grtc_sys_counter_capture_task_get(m_timestamp_cc_channel);

	NRF_DPPI_ENDPOINT_CLEAR(nrfy_grtc_task_address_get(NRF_GRTC, capture_task));
}

void nrf_802154_platform_timestamper_local_domain_connections_clear(uint32_t dppi_ch)
{
	/* Intentionally empty. */
}

bool nrf_802154_platform_timestamper_captured_timestamp_read(uint64_t *p_captured)
{
	/* @todo: check if this can be replaced with:
	 *
	 * z_nrf_grtc_timer_capture_read(m_timestamp_cc_channel, p_captured);
	 */
	if (nrf_grtc_sys_counter_cc_enable_check(NRF_GRTC, m_timestamp_cc_channel)) {
		return false;
	}

	*p_captured = nrfy_grtc_sys_counter_cc_get(NRF_GRTC, m_timestamp_cc_channel);
	return true;
}
