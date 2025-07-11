/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "nrf_802154_platform_sl_lptimer_grtc_hw_task.h"

#include <haly/nrfy_dppi.h>
#include <haly/nrfy_grtc.h>
#include <hal/nrf_ppib.h>

#include <platform/nrf_802154_platform_sl_lptimer.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>

#if defined(NRF54H_SERIES)

#include <hal/nrf_ipct.h>

/* To trigger RADIO.TASKS_{?} with GRTC.EVENT_COMPARE[#cc] the following connection chain must be
 * created:
 *    - starting from LOCAL (RADIO core) domain:
 *        {a} RADIO.TASKS_{?}  <-- DPPIC_020
 *        {b} DPPIC_020        <-- IPCT_radio
 *    - entering the GLOBAL "Main power" domain (G1):
 *        {c} IPCT_radio       <-- IPCT_130
 *        {d} IPCT_130         <-- DPPIC_130
 *        {e} DPPIC_130        <-- PPIB_130
 *        {f} PPIB_130         <-- PPIB_133
 *    - ending in the GLOBAL "Active power" domain (G2):
 *        {g} PPIB_133         <-- DPPIC_132
 *        {h} DPPIC_132        <-- GRTC.CC
 */

/* Peripherals used for hardware tasks - located in local domain (_L_) */
/*   - DPPIC_L : DPPIC020 (RADIOCORE.DPPIC020) */
/*   - IPCT_L : NRF_IPCT (RADIOCORE.IPCT) */
#define IPCT_L_HT_CHANNEL            2
#if defined(NRF54H20_ENGA_XXAA)
#define IPCT_L_SHORTS                IPCT_SHORTS_RECEIVE2_ACK2_Msk
#else
#define IPCT_L_SHORTS                IPCT_SHORTS_RECEIVE2_FLUSH2_Msk
#endif
#define IPCT_L_EVENT_RECEIVE         NRFX_CONCAT_2(NRF_IPCT_EVENT_RECEIVE_, IPCT_L_HT_CHANNEL)

/* Peripherals used for timestamping - located in global "Main power domain" (_G1_) */
/*   - IPCT_G1 : IPCT130_S */
#define IPCT_G1_INST                 NRF_IPCT130_S
#define IPCT_G1_HT_CHANNEL           2
#define IPCT_G1_TASK_SEND            NRFX_CONCAT_2(NRF_IPCT_TASK_SEND_, IPCT_G1_HT_CHANNEL)

/*   - DPPIC_G1 : DPPIC130_S */
/* The channel must be in the [0..7] range to satisfy the requirements for the dependent macros */
#define DPPIC_G1_INST                NRF_DPPIC130_S
#define DPPIC_G1_HT_CHANNEL          2

/*  - PPIB_G1 : PPIB130_S */
#define PPIB_G1_HT_CHANNEL           (DPPIC_G1_HT_CHANNEL + 8) /* hw-fixed dependency */

/* Peripherals used for timestamping - located in global "Active power domain" (_G2_) */
/*   - PPIB_G2 : PPIB133_S */
#define DPPIC_G2_INST                NRF_DPPIC132_S
#define PPIB_G2_HT_CHANNEL           (PPIB_G1_HT_CHANNEL - 8) /* hw-fixed dependency */

/*   - DPPIC_G2 : DPPIC132_S */
#define DPPIC_G2_HT_CHANNEL          (PPIB_G2_HT_CHANNEL + 0) /* hw-fixed dependency */

#if (PPIB_G1_HT_CHANNEL < 8) || (PPIB_G1_HT_CHANNEL > 15)
/* Only this range of channels can be connected to PPIB133 */
#error PPIB_G1_HT_CHANNEL is required to be in the [8..15] range
#endif

/* Ensure something similar to this is present in the DT:
 * &dppic130 {
 *	owned-channels = <2>;
 *	sink-channels = <2>;
 * };
 * where 2 corresponds to DPPIC_G1_HT_CHANNEL.
 */
#define _HT_DT_CHECK_DPPIC_CHANNEL(node_id, prop, idx) \
	(DT_PROP_BY_IDX(node_id, prop, idx) == DPPIC_G1_HT_CHANNEL) ||

#define HT_DT_HAS_RESERVED_DPPIC_CHANNEL		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(dppic130), sink_channels), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(dppic130), \
					  sink_channels, \
					  _HT_DT_CHECK_DPPIC_CHANNEL) false), \
		    (false))

BUILD_ASSERT(HT_DT_HAS_RESERVED_DPPIC_CHANNEL,
	     "The required DPPIC_G1 channel is not reserved");


/* Ensure something similar to this is present in the DT:
 * &dppic132 {
 *	owned-channels = <2>;
 *	source-channels = <2>;
 * where 2 corresponds to DPPIC_G2_HT_CHANNEL.
 * };
 */
#define _HT_DT_CHECK_DPPIC_G2_CHANNEL(node_id, prop, idx) \
	(DT_PROP_BY_IDX(node_id, prop, idx) == DPPIC_G2_HT_CHANNEL) ||

#define HT_DT_HAS_RESERVED_DPPIC_G2_CHANNEL		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(dppic132), source_channels), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(dppic132), \
					  source_channels, \
					  _HT_DT_CHECK_DPPIC_G2_CHANNEL) false), \
		    (false))

BUILD_ASSERT(HT_DT_HAS_RESERVED_DPPIC_G2_CHANNEL,
	     "The required DPPIC_G2 channel is not reserved");

/* Ensure something similar to this is present in the DT:
 * &cpurad_ipct {
 *	sink-channel-links = <2 13 2>;
 * };
 * where first 2 corresponds to IPCT_L_HT_CHANNEL.
 */
#define _HT_DT_CHECK_IPCT_L_LINK(node_id, prop, idx) \
	((DT_PROP_BY_IDX(node_id, prop, idx) == IPCT_L_HT_CHANNEL) && ((idx) % 3 == 0)) ||

#define HT_DT_HAS_RESERVED_IPCT_L_LINK		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(cpurad_ipct), sink_channel_links), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(cpurad_ipct), \
					  sink_channel_links, \
					  _HT_DT_CHECK_IPCT_L_LINK) false), \
		    (false))

/* NOTE: this not verifying the allocation as the device tree property is an
 * array of triplets that is difficult to separate using macros.
 */
BUILD_ASSERT(HT_DT_HAS_RESERVED_IPCT_L_LINK,
	     "The required IPCT_L link is not reserved");

/* Ensure something similar to this is present in the DT:
 * &ipct130 {
 *	source-channel-links = <2 3 2>;
 * };
 * where first 2 corresponds to IPCT_G1_HT_CHANNEL.
 */
#define _HT_DT_CHECK_IPCT_G1_LINK(node_id, prop, idx) \
	((DT_PROP_BY_IDX(node_id, prop, idx) == IPCT_G1_HT_CHANNEL) && ((idx) % 3 == 0)) ||


#define TS_DT_HAS_RESERVED_IPCT_G1_LINK		\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(ipct130), source_channel_links), \
		    (DT_FOREACH_PROP_ELEM(DT_NODELABEL(ipct130), \
					  source_channel_links, \
					  _HT_DT_CHECK_IPCT_G1_LINK) false), \
		    (false))

/* NOTE: this not verifying the allocation as the device tree property is an
 * array of triplets that is difficult to separate using macros.
 */
BUILD_ASSERT(TS_DT_HAS_RESERVED_IPCT_G1_LINK,
	     "The required IPCT_G1 link is not reserved");

void nrf_802154_platform_sl_lptimer_hw_task_cross_domain_connections_setup(uint32_t cc_channel)
{
	/* {c} IPCT_radio <-- IPCT_130
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring of the UICR->IPCMAP[] registers.
	 *
	 * Only enable auto confirmations on destination - IPCT_radio.
	 */
	nrf_ipct_shorts_enable(NRF_IPCT, IPCT_L_SHORTS);

	/* {d} IPCT_130 <-- DPPIC_130 */
	nrf_ipct_subscribe_set(IPCT_G1_INST, IPCT_G1_TASK_SEND, DPPIC_G1_HT_CHANNEL);

	/* {e} DPPIC_130 <-- PPIB_130
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring the UICR->DPPI.GLOBAL[].CH.LINK.SOURCE registers.
	 *
	 * Only enable relevant DPPIC_G1_INST channel.
	 */
	nrfy_dppi_channels_enable(DPPIC_G1_INST, 1UL << DPPIC_G1_HT_CHANNEL);

	/* {f} PPIB_130 <-- PPIB_133
	 * One of HW-fixed connections (channels [8..15] --> [0..7]), so nothing to do.
	 */

	/* {g} PPIB_133 <-- DPPIC_132
	 * It is assumed that this connection has already been made by SECURE-core as a result of
	 * configuring of the UICR->DPPI.GLOBAL[].CH.LINK.SINK registers.
	 */

	/* {h} DPPIC_132 <-- GRTC.CC */
	NRF_DPPI_ENDPOINT_SETUP(
		z_nrf_grtc_timer_compare_evt_address_get(cc_channel), DPPIC_G2_HT_CHANNEL);
}

void nrf_802154_platform_sl_lptimer_hw_task_cross_domain_connections_clear(void)
{
	/* @todo: implement */
}

void nrf_802154_platform_sl_lptimer_hw_task_local_domain_connections_setup(uint32_t dppi_ch,
									   uint32_t cc_channel)
{
	if (dppi_ch == NRF_802154_SL_HW_TASK_PPI_INVALID) {
		return;
	}

	nrf_ipct_event_clear(NRF_IPCT, IPCT_L_EVENT_RECEIVE);

	/* {a} RADIO.TASKS_{?} <-- DPPIC_020[dppi_ch]
	 * It is the responsibility of the user of this platform to make the {a} connection
	 * and pass the DPPI channel number as a parameter here.
	 */

	/* {b} DPPIC_020[dppi_ch] <-- IPCT_radio */
	nrf_ipct_publish_set(NRF_IPCT, IPCT_L_EVENT_RECEIVE, dppi_ch);

	/* {h} Enable relevant DPPIC_G2_INST channel. */
	nrfy_dppi_channels_enable(DPPIC_G2_INST, 1UL << DPPIC_G2_HT_CHANNEL);
}

void nrf_802154_platform_sl_lptimer_hw_task_local_domain_connections_clear(void)
{
	nrf_ipct_publish_clear(NRF_IPCT, IPCT_L_EVENT_RECEIVE);
	nrfy_dppi_channels_disable(DPPIC_G2_INST, 1UL << DPPIC_G2_HT_CHANNEL);
	nrf_ipct_event_clear(NRF_IPCT, IPCT_L_EVENT_RECEIVE);
}

#elif defined(NRF54L_SERIES)

#include <nrfx_ppib.h>
#include <nrfx_dppi.h>

/* To trigger GRTC.TASKS_CAPTURE[#cc] with RADIO.EVENT_{?}, the following connection chain must be
 * created:
 *    - starting from RADIO domain (_R_):
 *        {a} RADIO.EVENT_{?}  <-- DPPIC_10
 *        {b} DPPIC_10         <-- PPIB_11
 *    - crossing domain boundaries
 *        {c} PPIB_11          <-- PPIB_21
 *    - ending in the PERI domain (_P_):
 *        {d} PPIB_21          <-- DPPIC_20
 *        {e} DPPIC_20         <-- GRTC.CC
 */

#define INVALID_CHANNEL UINT8_MAX

static nrfx_dppi_t dppi20 = NRFX_DPPI_INSTANCE(20);
static nrfx_ppib_interconnect_t ppib11_21 = NRFX_PPIB_INTERCONNECT_INSTANCE(11, 21);
static uint8_t peri_dppi_ch = INVALID_CHANNEL;
static uint8_t peri_ppib_ch = INVALID_CHANNEL;

void nrf_802154_platform_sl_lptimer_hw_task_cross_domain_connections_setup(uint32_t cc_channel)
{
	nrfx_err_t err;

	err = nrfx_dppi_channel_alloc(&dppi20, &peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppib_channel_alloc(&ppib11_21, &peri_ppib_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	/* {c} PPIB_11 <-- PPIB_21
	 * One of HW-fixed connections, so nothing to do.
	 */

	/* {d} PPIB_21 <-- DPPIC_20 */
	NRF_DPPI_ENDPOINT_SETUP(
		nrfx_ppib_send_task_address_get(&ppib11_21.right, peri_ppib_ch), peri_dppi_ch);

	/* {e} DPPIC_20 <-- GRTC.CC */
	NRF_DPPI_ENDPOINT_SETUP(
		z_nrf_grtc_timer_compare_evt_address_get(cc_channel), peri_dppi_ch);
}

void nrf_802154_platform_sl_lptimer_hw_task_cross_domain_connections_clear(void)
{
	nrfx_err_t err;

	err = nrfx_ppib_channel_free(&ppib11_21, peri_ppib_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_dppi_channel_free(&dppi20, peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	peri_dppi_ch = INVALID_CHANNEL;
	peri_ppib_ch = INVALID_CHANNEL;
}

void nrf_802154_platform_sl_lptimer_hw_task_local_domain_connections_setup(uint32_t dppi_ch,
									   uint32_t cc_channel)
{
	nrfx_err_t err;

	if (dppi_ch == NRF_802154_SL_HW_TASK_PPI_INVALID) {
		return;
	}

	/* {a} RADIO.TASKS_{?} <-- DPPIC_10[dppi_ch]
	 * It is the responsibility of the user of this platform to make the {a} connection
	 * and pass the DPPI channel number as a parameter here.
	 */

	/* {b} DPPIC_10 <-- PPIB_11 */
	NRF_DPPI_ENDPOINT_SETUP(
		nrfx_ppib_receive_event_address_get(&ppib11_21.left, peri_ppib_ch), dppi_ch);

	err = nrfx_dppi_channel_enable(&dppi20, peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
}

void nrf_802154_platform_sl_lptimer_hw_task_local_domain_connections_clear(void)
{
	nrfx_err_t err;

	NRF_DPPI_ENDPOINT_CLEAR(
		nrfx_ppib_receive_event_address_get(&ppib11_21.left, peri_ppib_ch));

	err = nrfx_dppi_channel_disable(&dppi20, peri_dppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
}

#endif
