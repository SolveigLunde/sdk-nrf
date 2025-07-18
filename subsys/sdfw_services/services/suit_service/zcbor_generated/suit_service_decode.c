/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Generated using zcbor version 0.9.0
 * https://github.com/NordicSemiconductor/zcbor
 * Generated with a --default-max-qty of 3
 */

#include "suit_service_decode.h"
#include "zcbor_decode.h"
#include "zcbor_print.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if DEFAULT_MAX_QTY != 3
#error                                                                         \
    "The type file was generated with a different default_max_qty than this file"
#endif

#define log_result(state, result, func)                                        \
  do {                                                                         \
    if (!result) {                                                             \
      zcbor_trace_file(state);                                                 \
      zcbor_log("%s error: %s\r\n", func,                                      \
                zcbor_error_str(zcbor_peek_error(state)));                     \
    } else {                                                                   \
      zcbor_log("%s success\r\n", func);                                       \
    }                                                                          \
  } while (0)

static bool
decode_suit_missing_image_evt_nfy(zcbor_state_t *state,
                                  struct suit_missing_image_evt_nfy *result);
static bool
decode_suit_chunk_status_evt_nfy(zcbor_state_t *state,
                                 struct suit_chunk_status_evt_nfy *result);
static bool decode_suit_cache_info_entry(zcbor_state_t *state,
                                         struct suit_cache_info_entry *result);
static bool
decode_suit_trigger_update_req(zcbor_state_t *state,
                               struct suit_trigger_update_req *result);
static bool decode_suit_check_installed_component_digest_req(
    zcbor_state_t *state,
    struct suit_check_installed_component_digest_req *result);
static bool decode_suit_get_installed_manifest_info_req(
    zcbor_state_t *state, struct suit_get_installed_manifest_info_req *result);
static bool decode_suit_authenticate_manifest_req(
    zcbor_state_t *state, struct suit_authenticate_manifest_req *result);
static bool decode_suit_authorize_unsigned_manifest_req(
    zcbor_state_t *state, struct suit_authorize_unsigned_manifest_req *result);
static bool
decode_suit_authorize_seq_num_req(zcbor_state_t *state,
                                  struct suit_authorize_seq_num_req *result);
static bool decode_suit_check_component_compatibility_req(
    zcbor_state_t *state,
    struct suit_check_component_compatibility_req *result);
static bool decode_suit_get_supported_manifest_info_req(
    zcbor_state_t *state, struct suit_get_supported_manifest_info_req *result);
static bool decode_suit_authorize_process_dependency_req(
    zcbor_state_t *state, struct suit_authorize_process_dependency_req *result);
static bool
decode_suit_get_ipuc_info_req(zcbor_state_t *state,
                              struct suit_get_ipuc_info_req *result);
static bool
decode_suit_setup_write_ipuc_req(zcbor_state_t *state,
                                 struct suit_setup_write_ipuc_req *result);
static bool decode_suit_write_ipuc_req(zcbor_state_t *state,
                                       struct suit_write_ipuc_req *result);
static bool
decode_suit_get_manifest_var_req(zcbor_state_t *state,
                                 struct suit_get_manifest_var_req *result);
static bool
decode_suit_set_manifest_var_req(zcbor_state_t *state,
                                 struct suit_set_manifest_var_req *result);
static bool decode_suit_evt_sub_req(zcbor_state_t *state,
                                    struct suit_evt_sub_req *result);
static bool
decode_suit_chunk_enqueue_req(zcbor_state_t *state,
                              struct suit_chunk_enqueue_req *result);
static bool decode_suit_chunk_status_req(zcbor_state_t *state,
                                         struct suit_chunk_status_req *result);
static bool
decode_suit_invoke_confirm_req(zcbor_state_t *state,
                               struct suit_invoke_confirm_req *result);
static bool
decode_suit_trigger_update_rsp(zcbor_state_t *state,
                               struct suit_trigger_update_rsp *result);
static bool decode_suit_check_installed_component_digest_rsp(
    zcbor_state_t *state,
    struct suit_check_installed_component_digest_rsp *result);
static bool decode_suit_get_installed_manifest_info_rsp(
    zcbor_state_t *state, struct suit_get_installed_manifest_info_rsp *result);
static bool decode_suit_get_install_candidate_info_rsp(
    zcbor_state_t *state, struct suit_get_install_candidate_info_rsp *result);
static bool decode_suit_authenticate_manifest_rsp(
    zcbor_state_t *state, struct suit_authenticate_manifest_rsp *result);
static bool decode_suit_authorize_unsigned_manifest_rsp(
    zcbor_state_t *state, struct suit_authorize_unsigned_manifest_rsp *result);
static bool
decode_suit_authorize_seq_num_rsp(zcbor_state_t *state,
                                  struct suit_authorize_seq_num_rsp *result);
static bool decode_suit_check_component_compatibility_rsp(
    zcbor_state_t *state,
    struct suit_check_component_compatibility_rsp *result);
static bool decode_suit_get_supported_manifest_roles_rsp(
    zcbor_state_t *state, struct suit_get_supported_manifest_roles_rsp *result);
static bool decode_suit_get_supported_manifest_info_rsp(
    zcbor_state_t *state, struct suit_get_supported_manifest_info_rsp *result);
static bool decode_suit_authorize_process_dependency_rsp(
    zcbor_state_t *state, struct suit_authorize_process_dependency_rsp *result);
static bool
decode_suit_get_ipuc_count_rsp(zcbor_state_t *state,
                               struct suit_get_ipuc_count_rsp *result);
static bool
decode_suit_get_ipuc_info_rsp(zcbor_state_t *state,
                              struct suit_get_ipuc_info_rsp *result);
static bool
decode_suit_setup_write_ipuc_rsp(zcbor_state_t *state,
                                 struct suit_setup_write_ipuc_rsp *result);
static bool decode_suit_write_ipuc_rsp(zcbor_state_t *state,
                                       struct suit_write_ipuc_rsp *result);
static bool
decode_suit_get_manifest_var_rsp(zcbor_state_t *state,
                                 struct suit_get_manifest_var_rsp *result);
static bool
decode_suit_set_manifest_var_rsp(zcbor_state_t *state,
                                 struct suit_set_manifest_var_rsp *result);
static bool decode_suit_evt_sub_rsp(zcbor_state_t *state,
                                    struct suit_evt_sub_rsp *result);
static bool
decode_suit_chunk_enqueue_rsp(zcbor_state_t *state,
                              struct suit_chunk_enqueue_rsp *result);
static bool decode_suit_chunk_info_entry(zcbor_state_t *state,
                                         struct suit_chunk_info_entry *result);
static bool decode_suit_chunk_status_rsp(zcbor_state_t *state,
                                         struct suit_chunk_status_rsp *result);
static bool
decode_suit_boot_mode_read_rsp(zcbor_state_t *state,
                               struct suit_boot_mode_read_rsp *result);
static bool
decode_suit_invoke_confirm_rsp(zcbor_state_t *state,
                               struct suit_invoke_confirm_rsp *result);
static bool
decode_suit_boot_flags_reset_rsp(zcbor_state_t *state,
                                 struct suit_boot_flags_reset_rsp *result);
static bool decode_suit_foreground_dfu_required_rsp(
    zcbor_state_t *state, struct suit_foreground_dfu_required_rsp *result);
static bool decode_suit_nfy(zcbor_state_t *state, struct suit_nfy *result);
static bool decode_suit_rsp(zcbor_state_t *state, struct suit_rsp *result);
static bool decode_suit_req(zcbor_state_t *state, struct suit_req *result);

static bool
decode_suit_missing_image_evt_nfy(zcbor_state_t *state,
                                  struct suit_missing_image_evt_nfy *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (41)))) &&
         ((zcbor_bstr_decode(
             state, (&(*result).suit_missing_image_evt_nfy_resource_id)))) &&
         ((zcbor_uint32_decode(
             state,
             (&(*result).suit_missing_image_evt_nfy_stream_session_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_chunk_status_evt_nfy(zcbor_state_t *state,
                                 struct suit_chunk_status_evt_nfy *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (42)))) &&
      ((zcbor_uint32_decode(
          state, (&(*result).suit_chunk_status_evt_nfy_stream_session_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_cache_info_entry(zcbor_state_t *state,
                                         struct suit_cache_info_entry *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_decode(state, (&(*result).suit_cache_info_entry_addr)))) &&
      ((zcbor_uint32_decode(state,
                            (&(*result).suit_cache_info_entry_size)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_trigger_update_req(zcbor_state_t *state,
                               struct suit_trigger_update_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (1)))) &&
      ((zcbor_uint32_decode(state,
                            (&(*result).suit_trigger_update_req_addr)))) &&
      ((zcbor_uint32_decode(state,
                            (&(*result).suit_trigger_update_req_size)))) &&
      ((zcbor_list_start_decode(state) &&
        ((zcbor_multi_decode(
             0, 6,
             &(*result)
                  .suit_trigger_update_req_caches_suit_cache_info_entry_m_count,
             (zcbor_decoder_t *)decode_suit_cache_info_entry, state,
             (*&(*result)
                    .suit_trigger_update_req_caches_suit_cache_info_entry_m),
             sizeof(struct suit_cache_info_entry))) ||
         (zcbor_list_map_end_force_decode(state), false)) &&
        zcbor_list_end_decode(state))))));

  if (false) {
    /* For testing that the types of the arguments are correct.
     * A compiler error here means a bug in zcbor.
     */
    decode_suit_cache_info_entry(
        state,
        (*&(*result).suit_trigger_update_req_caches_suit_cache_info_entry_m));
  }

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_check_installed_component_digest_req(
    zcbor_state_t *state,
    struct suit_check_installed_component_digest_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (2)))) &&
       ((zcbor_bstr_decode(
           state,
           (&(*result)
                 .suit_check_installed_component_digest_req_component_id)))) &&
       ((zcbor_int32_decode(
           state,
           (&(*result).suit_check_installed_component_digest_req_alg_id)))) &&
       ((zcbor_bstr_decode(
           state,
           (&(*result).suit_check_installed_component_digest_req_digest)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_installed_manifest_info_req(
    zcbor_state_t *state, struct suit_get_installed_manifest_info_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (3)))) &&
       ((zcbor_bstr_decode(
           state,
           (&(*result)
                 .suit_get_installed_manifest_info_req_manifest_class_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authenticate_manifest_req(
    zcbor_state_t *state, struct suit_authenticate_manifest_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (10)))) &&
         ((zcbor_bstr_decode(
             state,
             (&(*result)
                   .suit_authenticate_manifest_req_manifest_component_id)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_authenticate_manifest_req_alg_id)))) &&
         ((zcbor_bstr_decode(
             state, (&(*result).suit_authenticate_manifest_req_key_id)))) &&
         ((zcbor_bstr_decode(
             state, (&(*result).suit_authenticate_manifest_req_signature)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_authenticate_manifest_req_data_addr)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_authenticate_manifest_req_data_size)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authorize_unsigned_manifest_req(
    zcbor_state_t *state, struct suit_authorize_unsigned_manifest_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (11)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result)
                .suit_authorize_unsigned_manifest_req_manifest_component_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_authorize_seq_num_req(zcbor_state_t *state,
                                  struct suit_authorize_seq_num_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (12)))) &&
         ((zcbor_bstr_decode(
             state,
             (&(*result).suit_authorize_seq_num_req_manifest_component_id)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_authorize_seq_num_req_command_seq)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_authorize_seq_num_req_seq_num)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_check_component_compatibility_req(
    zcbor_state_t *state,
    struct suit_check_component_compatibility_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (13)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result)
                .suit_check_component_compatibility_req_manifest_class_id)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result)
                .suit_check_component_compatibility_req_component_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_supported_manifest_info_req(
    zcbor_state_t *state, struct suit_get_supported_manifest_info_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (19)))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_get_supported_manifest_info_req_role)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authorize_process_dependency_req(
    zcbor_state_t *state,
    struct suit_authorize_process_dependency_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (21)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result)
                .suit_authorize_process_dependency_req_dependee_class_id)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result)
                .suit_authorize_process_dependency_req_dependent_class_id)))) &&
      ((zcbor_int32_decode(
          state,
          (&(*result).suit_authorize_process_dependency_req_seq_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_get_ipuc_info_req(zcbor_state_t *state,
                              struct suit_get_ipuc_info_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (31)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_get_ipuc_info_req_idx)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_setup_write_ipuc_req(zcbor_state_t *state,
                                 struct suit_setup_write_ipuc_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (32)))) &&
       ((zcbor_bstr_decode(
           state, (&(*result).suit_setup_write_ipuc_req_component_id)))) &&
       ((zcbor_bstr_decode(
           state, (&(*result).suit_setup_write_ipuc_req_encryption_info)))) &&
       ((zcbor_bstr_decode(
           state, (&(*result).suit_setup_write_ipuc_req_compression_info)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_write_ipuc_req(zcbor_state_t *state,
                                       struct suit_write_ipuc_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (33)))) &&
      ((zcbor_bstr_decode(state,
                          (&(*result).suit_write_ipuc_req_component_id)))) &&
      ((zcbor_uint32_decode(state, (&(*result).suit_write_ipuc_req_offset)))) &&
      ((zcbor_bool_decode(state,
                          (&(*result).suit_write_ipuc_req_last_chunk)))) &&
      ((zcbor_uint32_decode(state, (&(*result).suit_write_ipuc_req_addr)))) &&
      ((zcbor_uint32_decode(state, (&(*result).suit_write_ipuc_req_size)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_get_manifest_var_req(zcbor_state_t *state,
                                 struct suit_get_manifest_var_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (14)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_get_manifest_var_req_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_set_manifest_var_req(zcbor_state_t *state,
                                 struct suit_set_manifest_var_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (15)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_set_manifest_var_req_id)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_set_manifest_var_req_value)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_evt_sub_req(zcbor_state_t *state,
                                    struct suit_evt_sub_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (40)))) &&
       ((zcbor_bool_decode(state, (&(*result).suit_evt_sub_req_subscribe)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_chunk_enqueue_req(zcbor_state_t *state,
                              struct suit_chunk_enqueue_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (43)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_chunk_enqueue_req_stream_session_id)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_chunk_enqueue_req_chunk_id)))) &&
         ((zcbor_uint32_decode(state,
                               (&(*result).suit_chunk_enqueue_req_offset)))) &&
         ((zcbor_bool_decode(
             state, (&(*result).suit_chunk_enqueue_req_last_chunk)))) &&
         ((zcbor_uint32_decode(state,
                               (&(*result).suit_chunk_enqueue_req_addr)))) &&
         ((zcbor_uint32_decode(state,
                               (&(*result).suit_chunk_enqueue_req_size)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_chunk_status_req(zcbor_state_t *state,
                                         struct suit_chunk_status_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (44)))) &&
         ((zcbor_uint32_decode(
             state, (&(*result).suit_chunk_status_req_stream_session_id)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_invoke_confirm_req(zcbor_state_t *state,
                               struct suit_invoke_confirm_req *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (51)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_invoke_confirm_req_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_trigger_update_rsp(zcbor_state_t *state,
                               struct suit_trigger_update_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (1)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_trigger_update_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_check_installed_component_digest_rsp(
    zcbor_state_t *state,
    struct suit_check_installed_component_digest_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (2)))) &&
         ((zcbor_int32_decode(
             state,
             (&(*result).suit_check_installed_component_digest_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_installed_manifest_info_rsp(
    zcbor_state_t *state, struct suit_get_installed_manifest_info_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (3)))) &&
      ((zcbor_int32_decode(
          state, (&(*result).suit_get_installed_manifest_info_rsp_ret)))) &&
      ((zcbor_uint32_decode(
          state, (&(*result).suit_get_installed_manifest_info_rsp_seq_num)))) &&
      ((zcbor_list_start_decode(state) &&
        ((zcbor_multi_decode(
             0, 5,
             &(*result).suit_get_installed_manifest_info_rsp_semver_int_count,
             (zcbor_decoder_t *)zcbor_int32_decode, state,
             (*&(*result).suit_get_installed_manifest_info_rsp_semver_int),
             sizeof(int32_t))) ||
         (zcbor_list_map_end_force_decode(state), false)) &&
        zcbor_list_end_decode(state))) &&
      ((zcbor_int32_decode(
          state,
          (&(*result).suit_get_installed_manifest_info_rsp_digest_status)))) &&
      ((zcbor_int32_decode(
          state, (&(*result).suit_get_installed_manifest_info_rsp_alg_id)))) &&
      ((zcbor_bstr_decode(
          state, (&(*result).suit_get_installed_manifest_info_rsp_digest)))))));

  if (false) {
    /* For testing that the types of the arguments are correct.
     * A compiler error here means a bug in zcbor.
     */
    zcbor_int32_decode(
        state, (*&(*result).suit_get_installed_manifest_info_rsp_semver_int));
  }

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_install_candidate_info_rsp(
    zcbor_state_t *state, struct suit_get_install_candidate_info_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (4)))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_get_install_candidate_info_rsp_ret)))) &&
       ((zcbor_bstr_decode(
           state,
           (&(*result)
                 .suit_get_install_candidate_info_rsp_manifest_class_id)))) &&
       ((zcbor_uint32_decode(
           state, (&(*result).suit_get_install_candidate_info_rsp_seq_num)))) &&
       ((zcbor_list_start_decode(state) &&
         ((zcbor_multi_decode(
              0, 5,
              &(*result).suit_get_install_candidate_info_rsp_semver_int_count,
              (zcbor_decoder_t *)zcbor_int32_decode, state,
              (*&(*result).suit_get_install_candidate_info_rsp_semver_int),
              sizeof(int32_t))) ||
          (zcbor_list_map_end_force_decode(state), false)) &&
         zcbor_list_end_decode(state))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_get_install_candidate_info_rsp_alg_id)))) &&
       ((zcbor_bstr_decode(
           state, (&(*result).suit_get_install_candidate_info_rsp_digest)))))));

  if (false) {
    /* For testing that the types of the arguments are correct.
     * A compiler error here means a bug in zcbor.
     */
    zcbor_int32_decode(
        state, (*&(*result).suit_get_install_candidate_info_rsp_semver_int));
  }

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authenticate_manifest_rsp(
    zcbor_state_t *state, struct suit_authenticate_manifest_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (10)))) &&
         ((zcbor_int32_decode(
             state, (&(*result).suit_authenticate_manifest_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authorize_unsigned_manifest_rsp(
    zcbor_state_t *state, struct suit_authorize_unsigned_manifest_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (11)))) &&
         ((zcbor_int32_decode(
             state, (&(*result).suit_authorize_unsigned_manifest_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_authorize_seq_num_rsp(zcbor_state_t *state,
                                  struct suit_authorize_seq_num_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (12)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_authorize_seq_num_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_check_component_compatibility_rsp(
    zcbor_state_t *state,
    struct suit_check_component_compatibility_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (13)))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_check_component_compatibility_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_supported_manifest_roles_rsp(
    zcbor_state_t *state,
    struct suit_get_supported_manifest_roles_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (18)))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_get_supported_manifest_roles_rsp_ret)))) &&
       ((zcbor_list_start_decode(state) &&
         ((zcbor_multi_decode(
              0, 20,
              &(*result).suit_get_supported_manifest_roles_rsp_roles_int_count,
              (zcbor_decoder_t *)zcbor_int32_decode, state,
              (*&(*result).suit_get_supported_manifest_roles_rsp_roles_int),
              sizeof(int32_t))) ||
          (zcbor_list_map_end_force_decode(state), false)) &&
         zcbor_list_end_decode(state))))));

  if (false) {
    /* For testing that the types of the arguments are correct.
     * A compiler error here means a bug in zcbor.
     */
    zcbor_int32_decode(
        state, (*&(*result).suit_get_supported_manifest_roles_rsp_roles_int));
  }

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_get_supported_manifest_info_rsp(
    zcbor_state_t *state, struct suit_get_supported_manifest_info_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (19)))) &&
      ((zcbor_int32_decode(
          state, (&(*result).suit_get_supported_manifest_info_rsp_ret)))) &&
      ((zcbor_int32_decode(
          state, (&(*result).suit_get_supported_manifest_info_rsp_role)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result).suit_get_supported_manifest_info_rsp_vendor_id)))) &&
      ((zcbor_bstr_decode(
          state,
          (&(*result).suit_get_supported_manifest_info_rsp_class_id)))) &&
      ((zcbor_int32_decode(
          state,
          (&(*result)
                .suit_get_supported_manifest_info_rsp_downgrade_prevention_policy)))) &&
      ((zcbor_int32_decode(
          state,
          (&(*result)
                .suit_get_supported_manifest_info_rsp_independent_updateability_policy)))) &&
      ((zcbor_int32_decode(
          state,
          (&(*result)
                .suit_get_supported_manifest_info_rsp_signature_verification_policy)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_authorize_process_dependency_rsp(
    zcbor_state_t *state,
    struct suit_authorize_process_dependency_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (21)))) &&
       ((zcbor_int32_decode(
           state, (&(*result).suit_authorize_process_dependency_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_get_ipuc_count_rsp(zcbor_state_t *state,
                               struct suit_get_ipuc_count_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (30)))) &&
      ((zcbor_int32_decode(state, (&(*result).suit_get_ipuc_count_rsp_ret)))) &&
      ((zcbor_uint32_decode(state,
                            (&(*result).suit_get_ipuc_count_rsp_count)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_get_ipuc_info_rsp(zcbor_state_t *state,
                              struct suit_get_ipuc_info_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = ((
      (((zcbor_uint32_expect(state, (31)))) &&
       ((zcbor_int32_decode(state, (&(*result).suit_get_ipuc_info_rsp_ret)))) &&
       ((zcbor_bstr_decode(
           state, (&(*result).suit_get_ipuc_info_rsp_component_id)))) &&
       ((zcbor_int32_decode(state,
                            (&(*result).suit_get_ipuc_info_rsp_role)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_setup_write_ipuc_rsp(zcbor_state_t *state,
                                 struct suit_setup_write_ipuc_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (32)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_setup_write_ipuc_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_write_ipuc_rsp(zcbor_state_t *state,
                                       struct suit_write_ipuc_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (33)))) &&
         ((zcbor_int32_decode(state, (&(*result).suit_write_ipuc_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_get_manifest_var_rsp(zcbor_state_t *state,
                                 struct suit_get_manifest_var_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (14)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_get_manifest_var_rsp_ret)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_get_manifest_var_rsp_value)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_set_manifest_var_rsp(zcbor_state_t *state,
                                 struct suit_set_manifest_var_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (15)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_set_manifest_var_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_evt_sub_rsp(zcbor_state_t *state,
                                    struct suit_evt_sub_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (40)))) &&
         ((zcbor_int32_decode(state, (&(*result).suit_evt_sub_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_chunk_enqueue_rsp(zcbor_state_t *state,
                              struct suit_chunk_enqueue_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (43)))) &&
      ((zcbor_int32_decode(state, (&(*result).suit_chunk_enqueue_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_chunk_info_entry(zcbor_state_t *state,
                                         struct suit_chunk_info_entry *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_decode(
                    state, (&(*result).suit_chunk_info_entry_chunk_id)))) &&
                ((zcbor_uint32_decode(
                    state, (&(*result).suit_chunk_info_entry_status)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_chunk_status_rsp(zcbor_state_t *state,
                                         struct suit_chunk_status_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (44)))) &&
      ((zcbor_int32_decode(state, (&(*result).suit_chunk_status_rsp_ret)))) &&
      ((zcbor_list_start_decode(state) &&
        ((zcbor_multi_decode(
             0, 3,
             &(*result)
                  .suit_chunk_status_rsp_chunk_info_suit_chunk_info_entry_m_count,
             (zcbor_decoder_t *)decode_suit_chunk_info_entry, state,
             (*&(*result)
                    .suit_chunk_status_rsp_chunk_info_suit_chunk_info_entry_m),
             sizeof(struct suit_chunk_info_entry))) ||
         (zcbor_list_map_end_force_decode(state), false)) &&
        zcbor_list_end_decode(state))))));

  if (false) {
    /* For testing that the types of the arguments are correct.
     * A compiler error here means a bug in zcbor.
     */
    decode_suit_chunk_info_entry(
        state,
        (*&(*result).suit_chunk_status_rsp_chunk_info_suit_chunk_info_entry_m));
  }

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_boot_mode_read_rsp(zcbor_state_t *state,
                               struct suit_boot_mode_read_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((
      ((zcbor_uint32_expect(state, (50)))) &&
      ((zcbor_int32_decode(state, (&(*result).suit_boot_mode_read_rsp_ret)))) &&
      ((zcbor_uint32_decode(
          state, (&(*result).suit_boot_mode_read_rsp_boot_mode)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_invoke_confirm_rsp(zcbor_state_t *state,
                               struct suit_invoke_confirm_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (51)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_invoke_confirm_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool
decode_suit_boot_flags_reset_rsp(zcbor_state_t *state,
                                 struct suit_boot_flags_reset_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res = (((((zcbor_uint32_expect(state, (52)))) &&
                ((zcbor_int32_decode(
                    state, (&(*result).suit_boot_flags_reset_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_foreground_dfu_required_rsp(
    zcbor_state_t *state, struct suit_foreground_dfu_required_rsp *result) {
  zcbor_log("%s\r\n", __func__);

  bool res =
      (((((zcbor_uint32_expect(state, (53)))) &&
         ((zcbor_int32_decode(
             state, (&(*result).suit_foreground_dfu_required_rsp_ret)))))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_nfy(zcbor_state_t *state, struct suit_nfy *result) {
  zcbor_log("%s\r\n", __func__);
  bool int_res;

  bool res =
      (((zcbor_list_start_decode(state) &&
         ((((zcbor_union_start_code(state) &&
             (int_res =
                  ((((decode_suit_missing_image_evt_nfy(
                        state,
                        (&(*result)
                              .suit_nfy_msg_suit_missing_image_evt_nfy_m)))) &&
                    (((*result).suit_nfy_msg_choice =
                          suit_nfy_msg_suit_missing_image_evt_nfy_m_c),
                     true)) ||
                   (zcbor_union_elem_code(state) &&
                    (((decode_suit_chunk_status_evt_nfy(
                         state,
                         (&(*result)
                               .suit_nfy_msg_suit_chunk_status_evt_nfy_m)))) &&
                     (((*result).suit_nfy_msg_choice =
                           suit_nfy_msg_suit_chunk_status_evt_nfy_m_c),
                      true)))),
              zcbor_union_end_code(state), int_res)))) ||
          (zcbor_list_map_end_force_decode(state), false)) &&
         zcbor_list_end_decode(state))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_rsp(zcbor_state_t *state, struct suit_rsp *result) {
  zcbor_log("%s\r\n", __func__);
  bool int_res;

  bool res = (((
      zcbor_list_start_decode(state) &&
      ((((zcbor_union_start_code(state) &&
          (int_res =
               ((((decode_suit_trigger_update_rsp(
                     state,
                     (&(*result).suit_rsp_msg_suit_trigger_update_rsp_m)))) &&
                 (((*result).suit_rsp_msg_choice =
                       suit_rsp_msg_suit_trigger_update_rsp_m_c),
                  true)) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_check_installed_component_digest_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_check_installed_component_digest_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_check_installed_component_digest_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_installed_manifest_info_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_get_installed_manifest_info_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_installed_manifest_info_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_install_candidate_info_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_get_install_candidate_info_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_install_candidate_info_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authenticate_manifest_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_authenticate_manifest_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_authenticate_manifest_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_unsigned_manifest_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_authorize_unsigned_manifest_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_authorize_unsigned_manifest_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_seq_num_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_authorize_seq_num_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_authorize_seq_num_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_check_component_compatibility_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_check_component_compatibility_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_check_component_compatibility_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_supported_manifest_roles_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_get_supported_manifest_roles_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_supported_manifest_roles_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_supported_manifest_info_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_get_supported_manifest_info_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_supported_manifest_info_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_process_dependency_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_authorize_process_dependency_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_authorize_process_dependency_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_ipuc_count_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_get_ipuc_count_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_ipuc_count_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_ipuc_info_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_get_ipuc_info_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_ipuc_info_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_setup_write_ipuc_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_setup_write_ipuc_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_setup_write_ipuc_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_write_ipuc_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_write_ipuc_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_write_ipuc_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_manifest_var_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_get_manifest_var_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_get_manifest_var_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_set_manifest_var_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_set_manifest_var_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_set_manifest_var_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_evt_sub_rsp(
                      state, (&(*result).suit_rsp_msg_suit_evt_sub_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_evt_sub_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_chunk_enqueue_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_chunk_enqueue_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_chunk_enqueue_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_chunk_status_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_chunk_status_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_chunk_status_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_boot_mode_read_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_boot_mode_read_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_boot_mode_read_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_invoke_confirm_rsp(
                      state,
                      (&(*result).suit_rsp_msg_suit_invoke_confirm_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_invoke_confirm_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_boot_flags_reset_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_boot_flags_reset_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_boot_flags_reset_rsp_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_foreground_dfu_required_rsp(
                      state,
                      (&(*result)
                            .suit_rsp_msg_suit_foreground_dfu_required_rsp_m)))) &&
                  (((*result).suit_rsp_msg_choice =
                        suit_rsp_msg_suit_foreground_dfu_required_rsp_m_c),
                   true)))),
           zcbor_union_end_code(state), int_res)))) ||
       (zcbor_list_map_end_force_decode(state), false)) &&
      zcbor_list_end_decode(state))));

  log_result(state, res, __func__);
  return res;
}

static bool decode_suit_req(zcbor_state_t *state, struct suit_req *result) {
  zcbor_log("%s\r\n", __func__);
  bool int_res;

  bool res = (((
      zcbor_list_start_decode(state) &&
      ((((zcbor_union_start_code(state) &&
          (int_res =
               ((((decode_suit_trigger_update_req(
                     state,
                     (&(*result).suit_req_msg_suit_trigger_update_req_m)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_trigger_update_req_m_c),
                  true)) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_check_installed_component_digest_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_check_installed_component_digest_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_check_installed_component_digest_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_installed_manifest_info_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_get_installed_manifest_info_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_get_installed_manifest_info_req_m_c),
                   true))) ||
                (((zcbor_uint32_expect_union(state, (4)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_get_install_candidate_info_req_m_c),
                  true)) ||
                (((decode_suit_authenticate_manifest_req(
                     state,
                     (&(*result)
                           .suit_req_msg_suit_authenticate_manifest_req_m)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_authenticate_manifest_req_m_c),
                  true)) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_unsigned_manifest_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_authorize_unsigned_manifest_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_authorize_unsigned_manifest_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_seq_num_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_authorize_seq_num_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_authorize_seq_num_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_check_component_compatibility_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_check_component_compatibility_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_check_component_compatibility_req_m_c),
                   true))) ||
                (((zcbor_uint32_expect_union(state, (18)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_get_supported_manifest_roles_req_m_c),
                  true)) ||
                (((decode_suit_get_supported_manifest_info_req(
                     state,
                     (&(*result)
                           .suit_req_msg_suit_get_supported_manifest_info_req_m)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_get_supported_manifest_info_req_m_c),
                  true)) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_authorize_process_dependency_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_authorize_process_dependency_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_authorize_process_dependency_req_m_c),
                   true))) ||
                (((zcbor_uint32_expect_union(state, (30)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_get_ipuc_count_req_m_c),
                  true)) ||
                (((decode_suit_get_ipuc_info_req(
                     state,
                     (&(*result).suit_req_msg_suit_get_ipuc_info_req_m)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_get_ipuc_info_req_m_c),
                  true)) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_setup_write_ipuc_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_setup_write_ipuc_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_setup_write_ipuc_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_write_ipuc_req(
                      state,
                      (&(*result).suit_req_msg_suit_write_ipuc_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_write_ipuc_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_get_manifest_var_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_get_manifest_var_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_get_manifest_var_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_set_manifest_var_req(
                      state,
                      (&(*result)
                            .suit_req_msg_suit_set_manifest_var_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_set_manifest_var_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_evt_sub_req(
                      state, (&(*result).suit_req_msg_suit_evt_sub_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_evt_sub_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_chunk_enqueue_req(
                      state,
                      (&(*result).suit_req_msg_suit_chunk_enqueue_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_chunk_enqueue_req_m_c),
                   true))) ||
                (zcbor_union_elem_code(state) &&
                 (((decode_suit_chunk_status_req(
                      state,
                      (&(*result).suit_req_msg_suit_chunk_status_req_m)))) &&
                  (((*result).suit_req_msg_choice =
                        suit_req_msg_suit_chunk_status_req_m_c),
                   true))) ||
                (((zcbor_uint32_expect_union(state, (50)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_boot_mode_read_req_m_c),
                  true)) ||
                (((decode_suit_invoke_confirm_req(
                     state,
                     (&(*result).suit_req_msg_suit_invoke_confirm_req_m)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_invoke_confirm_req_m_c),
                  true)) ||
                (((zcbor_uint32_expect_union(state, (52)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_boot_flags_reset_req_m_c),
                  true)) ||
                (((zcbor_uint32_expect_union(state, (53)))) &&
                 (((*result).suit_req_msg_choice =
                       suit_req_msg_suit_foreground_dfu_required_req_m_c),
                  true))),
           zcbor_union_end_code(state), int_res)))) ||
       (zcbor_list_map_end_force_decode(state), false)) &&
      zcbor_list_end_decode(state))));

  log_result(state, res, __func__);
  return res;
}

int cbor_decode_suit_req(const uint8_t *payload, size_t payload_len,
                         struct suit_req *result, size_t *payload_len_out) {
  zcbor_state_t states[5];

  return zcbor_entry_function(payload, payload_len, (void *)result,
                              payload_len_out, states,
                              (zcbor_decoder_t *)decode_suit_req,
                              sizeof(states) / sizeof(zcbor_state_t), 1);
}

int cbor_decode_suit_rsp(const uint8_t *payload, size_t payload_len,
                         struct suit_rsp *result, size_t *payload_len_out) {
  zcbor_state_t states[5];

  return zcbor_entry_function(payload, payload_len, (void *)result,
                              payload_len_out, states,
                              (zcbor_decoder_t *)decode_suit_rsp,
                              sizeof(states) / sizeof(zcbor_state_t), 1);
}

int cbor_decode_suit_nfy(const uint8_t *payload, size_t payload_len,
                         struct suit_nfy *result, size_t *payload_len_out) {
  zcbor_state_t states[4];

  return zcbor_entry_function(payload, payload_len, (void *)result,
                              payload_len_out, states,
                              (zcbor_decoder_t *)decode_suit_nfy,
                              sizeof(states) / sizeof(zcbor_state_t), 1);
}
