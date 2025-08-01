/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Generated using zcbor version 0.8.1
 * https://github.com/NordicSemiconductor/zcbor
 * Generated with a --default-max-qty of 10
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "zcbor_encode.h"
#include "agnss_encode.h"
#include "zcbor_print.h"

#if DEFAULT_MAX_QTY != 10
#error "The type file was generated with a different default_max_qty than this file"
#endif

static bool encode_repeated_agnss_req_types(zcbor_state_t *state,
					    const struct agnss_req_types_r *input);
static bool encode_repeated_agnss_req_eci(zcbor_state_t *state, const struct agnss_req_eci *input);
static bool encode_repeated_agnss_req_filtered(zcbor_state_t *state,
					       const struct agnss_req_filtered *input);
static bool encode_repeated_agnss_req_mask(zcbor_state_t *state,
					   const struct agnss_req_mask *input);
static bool encode_repeated_agnss_req_mcc(zcbor_state_t *state, const struct agnss_req_mcc *input);
static bool encode_repeated_agnss_req_mnc(zcbor_state_t *state, const struct agnss_req_mnc *input);
static bool encode_repeated_agnss_req_rsrp(zcbor_state_t *state,
					   const struct agnss_req_rsrp *input);
static bool encode_repeated_agnss_req_tac(zcbor_state_t *state, const struct agnss_req_tac *input);
static bool encode_agnss_req(zcbor_state_t *state, const struct agnss_req *input);

static bool encode_repeated_agnss_req_types(zcbor_state_t *state,
					    const struct agnss_req_types_r *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result =
		((((zcbor_uint32_put(state, (1)))) &&
		  (zcbor_list_start_encode(state, 13) &&
		   ((zcbor_multi_encode_minmax(1, 13, &(*input).agnss_req_types_int_count,
					       (zcbor_encoder_t *)zcbor_int32_encode, state,
					       (&(*input).agnss_req_types_int), sizeof(int32_t))) ||
		    (zcbor_list_map_end_force_encode(state), false)) &&
		   zcbor_list_end_encode(state, 13))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_eci(zcbor_state_t *state, const struct agnss_req_eci *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (2)))) &&
			    (zcbor_uint32_encode(state, (&(*input).agnss_req_eci)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_filtered(zcbor_state_t *state,
					       const struct agnss_req_filtered *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (3)))) &&
			    (zcbor_bool_encode(state, (&(*input).agnss_req_filtered)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_mask(zcbor_state_t *state, const struct agnss_req_mask *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (4)))) &&
			    (zcbor_uint32_encode(state, (&(*input).agnss_req_mask)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_mcc(zcbor_state_t *state, const struct agnss_req_mcc *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (5)))) &&
			    (zcbor_uint32_encode(state, (&(*input).agnss_req_mcc)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_mnc(zcbor_state_t *state, const struct agnss_req_mnc *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (6)))) &&
			    (zcbor_uint32_encode(state, (&(*input).agnss_req_mnc)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_rsrp(zcbor_state_t *state, const struct agnss_req_rsrp *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (7)))) &&
			    (zcbor_int32_encode(state, (&(*input).agnss_req_rsrp)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_repeated_agnss_req_tac(zcbor_state_t *state, const struct agnss_req_tac *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result = ((((zcbor_uint32_put(state, (8)))) &&
			    (zcbor_uint32_encode(state, (&(*input).agnss_req_tac)))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

static bool encode_agnss_req(zcbor_state_t *state, const struct agnss_req *input)
{
	zcbor_log("%s\r\n", __func__);

	bool tmp_result =
		(((zcbor_map_start_encode(state, 8) &&
		   (((!(*input).agnss_req_types_present ||
		      encode_repeated_agnss_req_types(state, (&(*input).agnss_req_types))) &&
		     (!(*input).agnss_req_eci_present ||
		      encode_repeated_agnss_req_eci(state, (&(*input).agnss_req_eci))) &&
		     (!(*input).agnss_req_filtered_present ||
		      encode_repeated_agnss_req_filtered(state, (&(*input).agnss_req_filtered))) &&
		     (!(*input).agnss_req_mask_present ||
		      encode_repeated_agnss_req_mask(state, (&(*input).agnss_req_mask))) &&
		     (!(*input).agnss_req_mcc_present ||
		      encode_repeated_agnss_req_mcc(state, (&(*input).agnss_req_mcc))) &&
		     (!(*input).agnss_req_mnc_present ||
		      encode_repeated_agnss_req_mnc(state, (&(*input).agnss_req_mnc))) &&
		     (!(*input).agnss_req_rsrp_present ||
		      encode_repeated_agnss_req_rsrp(state, (&(*input).agnss_req_rsrp))) &&
		     (!(*input).agnss_req_tac_present ||
		      encode_repeated_agnss_req_tac(state, (&(*input).agnss_req_tac)))) ||
		    (zcbor_list_map_end_force_encode(state), false)) &&
		   zcbor_map_end_encode(state, 8))));

	if (!tmp_result) {
		zcbor_trace_file(state);
		zcbor_log("%s error: %s\r\n", __func__, zcbor_error_str(zcbor_peek_error(state)));
	} else {
		zcbor_log("%s success\r\n", __func__);
	}

	return tmp_result;
}

int cbor_encode_agnss_req(uint8_t *payload, size_t payload_len, const struct agnss_req *input,
			  size_t *payload_len_out)
{
	zcbor_state_t states[4];

	return zcbor_entry_function(payload, payload_len, (void *)input, payload_len_out, states,
				    (zcbor_decoder_t *)encode_agnss_req,
				    sizeof(states) / sizeof(zcbor_state_t), 1);
}
