/**
 *
 * @file
 *
 * @copyright Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <psa/crypto.h>
#include <psa/crypto_values.h>
#include <sxsymcrypt/aead.h>
#include <sxsymcrypt/aes.h>
#include <sxsymcrypt/chachapoly.h>
#include <sxsymcrypt/internal.h>
#include <sxsymcrypt/keyref.h>
#include <cracen/mem_helpers.h>
#include <cracen/statuscodes.h>
#include <zephyr/sys/__assert.h>
#include "common.h"

#define CCM_HEADER_MAX_LENGTH 26

/*
 * This function assumes it is given a valid algorithm for aead.
 */
static psa_key_type_t alg_to_key_type(psa_algorithm_t alg)
{

	switch (alg) {
	case PSA_ALG_GCM:
		IF_ENABLED(PSA_NEED_CRACEN_GCM_AES, (return PSA_KEY_TYPE_AES));
		break;
	case PSA_ALG_CCM:
		IF_ENABLED(PSA_NEED_CRACEN_CCM_AES, (return PSA_KEY_TYPE_AES));
		break;
	case PSA_ALG_CHACHA20_POLY1305:
		IF_ENABLED(PSA_NEED_CRACEN_CHACHA20_POLY1305, (return PSA_KEY_TYPE_CHACHA20));
		break;
	}

	return PSA_KEY_TYPE_NONE;
}

static bool is_key_type_supported(psa_algorithm_t alg, const psa_key_attributes_t *attributes)
{
	return psa_get_key_type(attributes) == alg_to_key_type(alg);
}

static bool is_nonce_length_supported(psa_algorithm_t alg, size_t nonce_length)
{
	switch (alg) {
	case PSA_ALG_GCM:
		IF_ENABLED(PSA_NEED_CRACEN_GCM_AES,
			   (return nonce_length == SX_GCM_IV_SZ));
	case PSA_ALG_CHACHA20_POLY1305:
		IF_ENABLED(PSA_NEED_CRACEN_CHACHA20_POLY1305,
			   (return nonce_length == SX_CHACHAPOLY_IV_SZ));
	case PSA_ALG_CCM:
		IF_ENABLED(PSA_NEED_CRACEN_CCM_AES,
			   (return sx_aead_aesccm_nonce_size_is_valid(nonce_length)));
		break;
	}

	return false;
}

static uint8_t get_tag_size(psa_algorithm_t alg, size_t key_buffer_size)
{
	return PSA_AEAD_TAG_LENGTH(alg_to_key_type(PSA_ALG_AEAD_WITH_DEFAULT_LENGTH_TAG(alg)),
				   PSA_BYTES_TO_BITS(key_buffer_size), alg);
}

/*
 * There are three supported algorithms GCM, CCM, and CHACHA20_POLY1305.
 *
 * GCM and CCM have a block size of 16, AKA
 * PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES)
 *
 * ChaCha20 operates internally with 64 byte blocks, so does the sxsymcrypt
 * driver
 */
static uint8_t get_block_size(psa_algorithm_t alg)
{
	switch (alg) {
	case PSA_ALG_GCM:
		IF_ENABLED(PSA_NEED_CRACEN_GCM_AES, (return 16));
		break;
	case PSA_ALG_CCM:
		IF_ENABLED(PSA_NEED_CRACEN_CCM_AES, (return 16));
		break;
	case PSA_ALG_CHACHA20_POLY1305:
		IF_ENABLED(PSA_NEED_CRACEN_CHACHA20_POLY1305, (return 64));
		break;
	}

	__ASSERT_NO_MSG(false);
	return 0;
}

static psa_status_t process_on_hw(cracen_aead_operation_t *operation)
{
	int sx_status = sx_aead_save_state(&operation->ctx);

	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}

	sx_status = sx_aead_wait(&operation->ctx);
	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}
	operation->context_state = CRACEN_CONTEXT_INITIALIZED;
	return silex_statuscodes_to_psa(sx_status);
}

static psa_status_t initialize_ctx(cracen_aead_operation_t *operation)
{
	int sx_status = SX_ERR_INCOMPATIBLE_HW;

	/* If the nonce_length is wrong then we must be in a bad state */
	if (!is_nonce_length_supported(operation->alg, operation->nonce_length)) {
		return PSA_ERROR_BAD_STATE;
	}

	switch (operation->alg) {
	case PSA_ALG_GCM:
		if (IS_ENABLED(PSA_NEED_CRACEN_GCM_AES)) {
			sx_status = operation->dir == CRACEN_DECRYPT
					    ? sx_aead_create_aesgcm_dec(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->tag_size)
					    : sx_aead_create_aesgcm_enc(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->tag_size);
		}
		break;
	case PSA_ALG_CCM:
		if (IS_ENABLED(PSA_NEED_CRACEN_CCM_AES)) {
			sx_status = operation->dir == CRACEN_DECRYPT
					    ? sx_aead_create_aesccm_dec(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->nonce_length,
						      operation->tag_size, operation->ad_length,
						      operation->plaintext_length)
					    : sx_aead_create_aesccm_enc(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->nonce_length,
						      operation->tag_size, operation->ad_length,
						      operation->plaintext_length);
		}
		break;
	case PSA_ALG_CHACHA20_POLY1305:
		if (IS_ENABLED(PSA_NEED_CRACEN_CHACHA20_POLY1305)) {
			sx_status = operation->dir == CRACEN_DECRYPT
					    ? sx_aead_create_chacha20poly1305_dec(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->tag_size)
					    : sx_aead_create_chacha20poly1305_enc(
						      &operation->ctx, &operation->keyref,
						      operation->nonce, operation->tag_size);
		}
		break;
	default:
		sx_status = SX_ERR_INCOMPATIBLE_HW;
		break;
	}

	return silex_statuscodes_to_psa(sx_status);
}

static psa_status_t initialize_or_resume_context(cracen_aead_operation_t *operation)
{
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

	switch (operation->context_state) {
	case CRACEN_NOT_INITIALIZED:
		status = initialize_ctx(operation);

		operation->context_state = CRACEN_HW_RESERVED;
		break;
	case CRACEN_CONTEXT_INITIALIZED:
		status = silex_statuscodes_to_psa(sx_aead_resume_state(&operation->ctx));

		operation->context_state = CRACEN_HW_RESERVED;
		break;
	case CRACEN_HW_RESERVED:
		status = PSA_SUCCESS;
	}

	return status;
}

static void cracen_writebe(uint8_t *out, uint64_t data, uint16_t targetsz)
{
	uint_fast16_t i;

	for (i = 0; i < targetsz; i++) {
		out[(targetsz - 1) - i] = (data >> (i * 8)) & 0xFF;
	}
}

static void create_aead_ccmheader(cracen_aead_operation_t *operation,
				  uint8_t header[static CCM_HEADER_MAX_LENGTH],
				  size_t *header_length)
{
	uint8_t flags;
	size_t m, l;

	/* RFC3610 paragraph 2.2 defines the formatting of the first block.
	 * M, CCM TAG size is one of {4,6,8,10,12,14,16}, CCM* not supported
	 * (MAC size 0)
	 * L must be between 2 and 8.
	 * Nonce size should be between 7 and 13 bytes.
	 * The first block contains:
	 *  byte  [0]           the flags byte (see below)
	 *  bytes [1,1+nonce.len]   nonce
	 *  bytes [2+nonce.len, 16] message length
	 *
	 *  The flags byte has the following bit fields:
	 *    [7]   = 0 Reserved
	 *    [6]   = 1 if ad data provided, else 0
	 *    [3:5] = authentication tag size, encoded as (tagsz-2)/2
	 *              only multiples of 2 between 2 and 16 are allowed.
	 *    [0:2] = length field size minus 1.
	 **/
	l = 15 - operation->nonce_length;

	flags = (operation->ad_length > 0) ? (1 << 6) : 0;
	m = (operation->tag_size - 2) / 2;

	flags |= (m & 0x7) << 3;
	flags |= ((l - 1) & 0x7);
	header[0] = flags;

	memcpy(&header[1], (void *)operation->nonce, operation->nonce_length);

	cracen_writebe(&(header[1 + operation->nonce_length]), operation->plaintext_length, l);

	*header_length = 16;
	/*
	 * If there is additional authentication data, encode the size into
	 * bytes [16, 17/21/25] depending on the length
	 */
	if (operation->ad_length > 0) {
		if (operation->ad_length < 0xFF00) {
			cracen_writebe(&header[16], operation->ad_length, 2);
			*header_length += 2;
		} else if (operation->ad_length <= 0xFFFFFFFF) {
			header[16] = 0xFF;
			header[17] = 0xFE;
			cracen_writebe(&header[18], operation->ad_length, 4);
			*header_length += 6;
		} else {
			header[16] = 0xFF;
			header[17] = 0xFF;
			cracen_writebe(&header[18], operation->ad_length, 8);
			*header_length += 10;
		}
	}
}

static psa_status_t setup(cracen_aead_operation_t *operation, enum cipher_operation dir,
			  const psa_key_attributes_t *attributes, const uint8_t *key_buffer,
			  size_t key_buffer_size, psa_algorithm_t alg)
{
	/*
	 * All algorithms in PSA Crypto 1.1.0 (PSA_ALG_CCM, PSA_ALG_GCM,
	 * and PSA_ALG_CHACHA20_POLY1305) are supported by this driver and
	 * the Oberon PSA Core does input validation that the algorithm is
	 * an AEAD algorithm so we omit input-validation of alg here.
	 *
	 * The operation must be inactive, but this is validated by the
	 * PSA Core so we do not need to validate it here.
	 */

	/*
	 * It is not clear if the Oberon PSA Core does input validation of
	 * the key attributes or not and this driver will later assume the
	 * key type is appropriate so we check it here to be safe.
	 */
	if (!is_key_type_supported(PSA_ALG_AEAD_WITH_DEFAULT_LENGTH_TAG(alg), attributes)) {
		return PSA_ERROR_NOT_SUPPORTED;
	}

	/*
	 * Copy the key into the operation struct as it is not guaranteed to be
	 * valid longer than the function call
	 */

	if (key_buffer_size > sizeof(operation->key_buffer)) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}

	memcpy(operation->key_buffer, key_buffer, key_buffer_size);

	psa_status_t status = cracen_load_keyref(attributes, operation->key_buffer, key_buffer_size,
						 &operation->keyref);
	if (status != PSA_SUCCESS) {
		return status;
	}

	operation->alg = PSA_ALG_AEAD_WITH_DEFAULT_LENGTH_TAG(alg);
	operation->dir = dir;
	operation->tag_size = get_tag_size(alg, key_buffer_size);

	/*
	 * At this point the nonce is not known, which is required to
	 * create a sxsymcrypt context, therefore all values are stored
	 * in the psa operation context.
	 */
	return status;
}

static psa_status_t cracen_feed_data_to_hw(cracen_aead_operation_t *operation, const uint8_t *input,
					   size_t input_length, uint8_t *output, bool is_ad_update)
{
	int sx_status;
	psa_status_t psa_status = PSA_ERROR_CORRUPTION_DETECTED;

	psa_status = initialize_or_resume_context(operation);
	if (psa_status) {
		return psa_status;
	}
	if (is_ad_update) {
		sx_status = sx_aead_feed_aad(&operation->ctx, input, input_length);
	} else {
		sx_status = sx_aead_crypt(&operation->ctx, input, input_length, output);
	}

	return silex_statuscodes_to_psa(sx_status);
}

psa_status_t cracen_aead_encrypt_setup(cracen_aead_operation_t *operation,
				       const psa_key_attributes_t *attributes,
				       const uint8_t *key_buffer, size_t key_buffer_size,
				       psa_algorithm_t alg)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	return setup(operation, CRACEN_ENCRYPT, attributes, key_buffer, key_buffer_size, alg);
#endif
}

psa_status_t cracen_aead_decrypt_setup(cracen_aead_operation_t *operation,
				       const psa_key_attributes_t *attributes,
				       const uint8_t *key_buffer, size_t key_buffer_size,
				       psa_algorithm_t alg)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	return setup(operation, CRACEN_DECRYPT, attributes, key_buffer, key_buffer_size, alg);
#endif
}

static psa_status_t set_nonce(cracen_aead_operation_t *operation, const uint8_t *nonce,
			      size_t nonce_length)
{
	if (!is_nonce_length_supported(operation->alg, nonce_length)) {
		return PSA_ERROR_NOT_SUPPORTED;
	}

	memcpy(operation->nonce, nonce, nonce_length);
	operation->nonce_length = nonce_length;

	return PSA_SUCCESS;
}

psa_status_t cracen_aead_set_nonce(cracen_aead_operation_t *operation, const uint8_t *nonce,
				   size_t nonce_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	psa_status_t status;

	status = set_nonce(operation, nonce, nonce_length);
	if (status != PSA_SUCCESS) {
		return status;
	}

	/* Create and feed the CCM header as additional data. It needs to be fed first
	 * (before actual additional data) so do it before the first cracen_aead_update_ad().
	 */
	if (IS_ENABLED(PSA_NEED_CRACEN_CCM_AES) && operation->alg == PSA_ALG_CCM) {
		uint8_t ccm_header[CCM_HEADER_MAX_LENGTH];
		size_t ccm_header_length;

		create_aead_ccmheader(operation, ccm_header, &ccm_header_length);
		return cracen_aead_update_ad(operation, ccm_header, ccm_header_length);
	}

	return PSA_SUCCESS;
#endif
}

static void set_lengths(cracen_aead_operation_t *operation, size_t ad_length,
			size_t plaintext_length)
{
	operation->ad_length = ad_length;
	operation->plaintext_length = plaintext_length;
}

psa_status_t cracen_aead_set_lengths(cracen_aead_operation_t *operation, size_t ad_length,
				     size_t plaintext_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	set_lengths(operation, ad_length, plaintext_length);
	return PSA_SUCCESS;
#endif
}

static psa_status_t cracen_aead_update_internal(cracen_aead_operation_t *operation,
						const uint8_t *input, size_t input_length,
						uint8_t *output, size_t output_size,
						size_t *output_length, bool is_ad_update)
{
	psa_status_t psa_status = PSA_ERROR_CORRUPTION_DETECTED;
	size_t blk_bytes = 0;
	size_t out_bytes = 0;

	if (input_length == 0) {
		return PSA_SUCCESS;
	}

	if (operation->unprocessed_input_bytes || input_length < get_block_size(operation->alg)) {
		uint8_t remaining_bytes =
			get_block_size(operation->alg) - operation->unprocessed_input_bytes;
		if (input_length <= remaining_bytes) {
			memcpy(operation->unprocessed_input + operation->unprocessed_input_bytes,
			       input, input_length);
			operation->unprocessed_input_bytes += input_length;
			/* The output_length can be NULL when we process the additional data because
			 * the value is not needed by any of the supported algorithms.
			 */
			if (output_length != NULL) {
				*output_length = 0;
			}
			return PSA_SUCCESS;
		}

		memcpy(operation->unprocessed_input + operation->unprocessed_input_bytes, input,
		       remaining_bytes);
		input += remaining_bytes;
		input_length -= remaining_bytes;
		operation->unprocessed_input_bytes += remaining_bytes;
	}

	if (operation->unprocessed_input_bytes == get_block_size(operation->alg)) {
		if (!is_ad_update && output_size < operation->unprocessed_input_bytes) {
			return PSA_ERROR_BUFFER_TOO_SMALL;
		}

		psa_status = cracen_feed_data_to_hw(operation, operation->unprocessed_input,
						    operation->unprocessed_input_bytes, output,
						    is_ad_update);
		if (psa_status) {
			return psa_status;
		}
		if (!is_ad_update) {
			output = output + operation->unprocessed_input_bytes;
		}

		out_bytes = operation->unprocessed_input_bytes;
		operation->unprocessed_input_bytes = 0;
	}

	/* Clamp input length to a multiple of the block size. */
	blk_bytes = input_length & ~(get_block_size(operation->alg) - 1);

	/* For CCM, in multi-part mode sxsymcrypt needs a chunk of input data to produce a tag
	 * therefore we buffer the last block until finish will be called.
	 * blk_bytes tracks the amount of block-sized input that will be
	 * processed immediately. So to buffer the input and prevent processing
	 * we subtract one block from blk_bytes.
	 */
	if (IS_ENABLED(PSA_NEED_CRACEN_CCM_AES) &&
	    operation->alg == PSA_ALG_CCM && input_length != 0 && blk_bytes == input_length) {
		blk_bytes -= get_block_size(operation->alg);
	}

	if (blk_bytes > 0) {
		if (!is_ad_update && (output_size < (blk_bytes + out_bytes))) {
			return PSA_ERROR_BUFFER_TOO_SMALL;
		}

		/* Feed the full blocks to the device. */
		psa_status =
			cracen_feed_data_to_hw(operation, input, blk_bytes, output, is_ad_update);
		if (psa_status) {
			return psa_status;
		}

		input += blk_bytes;
		input_length -= blk_bytes;
		out_bytes += blk_bytes;
	}

	/* Only process on HW if data has been fed */
	if (out_bytes != 0) {
		psa_status = process_on_hw(operation);
		if (psa_status) {
			return psa_status;
		}
		if (output_length) {
			*output_length = out_bytes;
		}
	}

	/* Copy remaining bytes to be encrypted later. */
	size_t remaining_bytes = input_length;

	if (remaining_bytes) {
		memcpy(operation->unprocessed_input, input, remaining_bytes);
		operation->unprocessed_input_bytes = remaining_bytes;
	}

	return PSA_SUCCESS;
}

psa_status_t cracen_aead_update_ad(cracen_aead_operation_t *operation, const uint8_t *input,
				   size_t input_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	return cracen_aead_update_internal(operation, input, input_length, NULL, 0, NULL, true);
#endif
}

psa_status_t cracen_aead_update(cracen_aead_operation_t *operation, const uint8_t *input,
				size_t input_length, uint8_t *output, size_t output_size,
				size_t *output_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	/*
	 * Even if no plain/ciphertext is provided we still wanna have one block
	 * of AD buffered before creating/verifying the tag
	 */
	if (input_length == 0) {
		*output_length = 0;
		return PSA_SUCCESS;
	}

	/* If there is unprocessed "additional data", then feed it before
	 * feeding the plaintext.
	 */
	if (operation->unprocessed_input_bytes && !operation->ad_finished) {
		psa_status_t status =
			cracen_feed_data_to_hw(operation, operation->unprocessed_input,
					       operation->unprocessed_input_bytes, NULL, true);
		if (status != PSA_SUCCESS) {
			return status;
		}

		status = process_on_hw(operation);
		if (status != PSA_SUCCESS) {
			return status;
		}

		operation->unprocessed_input_bytes = 0;
	}

	operation->ad_finished = true;

	return cracen_aead_update_internal(operation, input, input_length, output, output_size,
					   output_length, false);
#endif
}

static psa_status_t finalize_aead_encryption(cracen_aead_operation_t *operation, uint8_t *tag,
					     size_t tag_size, size_t *tag_length)
{
	int sx_status;

	if (tag_size < operation->tag_size) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	sx_status = sx_aead_produce_tag(&operation->ctx, tag);
	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}

	sx_status = sx_aead_wait(&operation->ctx);
	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}

	*tag_length = operation->tag_size;

	return cracen_aead_abort(operation);
}

psa_status_t cracen_aead_finish(cracen_aead_operation_t *operation, uint8_t *ciphertext,
				size_t ciphertext_size, size_t *ciphertext_length, uint8_t *tag,
				size_t tag_size, size_t *tag_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

	if (operation->ad_finished && (ciphertext_size < operation->unprocessed_input_bytes)) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	if (operation->unprocessed_input_bytes) {
		status = cracen_feed_data_to_hw(operation, operation->unprocessed_input,
						operation->unprocessed_input_bytes, ciphertext,
						!operation->ad_finished);
		if (status != PSA_SUCCESS) {
			return status;
		}
		if (operation->ad_finished) {
			*ciphertext_length = operation->unprocessed_input_bytes;
		}
	} else {
		/* In this case plaintext and AD has length 0 and we don't have a context.
		 * Initialize it here, so we can produce the tag.
		 */
		initialize_or_resume_context(operation);
	}

	return finalize_aead_encryption(operation, tag, tag_size, tag_length);
#endif
}

static psa_status_t finalize_aead_decryption(cracen_aead_operation_t *operation, const uint8_t *tag)
{
	int sx_status;

	sx_status = sx_aead_verify_tag(&operation->ctx, tag);
	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}

	sx_status = sx_aead_wait(&operation->ctx);
	if (sx_status) {
		return silex_statuscodes_to_psa(sx_status);
	}

	return cracen_aead_abort(operation);
}

psa_status_t cracen_aead_verify(cracen_aead_operation_t *operation, uint8_t *plaintext,
				size_t plaintext_size, size_t *plaintext_length, const uint8_t *tag,
				size_t tag_length)
{
#ifdef CONFIG_SOC_NRF54LM20A
	return PSA_ERROR_NOT_SUPPORTED;
#else
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

	if (operation->ad_finished && plaintext_size < operation->unprocessed_input_bytes) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	if (operation->unprocessed_input_bytes) {
		status = cracen_feed_data_to_hw(operation, operation->unprocessed_input,
						operation->unprocessed_input_bytes, plaintext,
						!operation->ad_finished);
		if (status != PSA_SUCCESS) {
			return status;
		}
		if (operation->ad_finished) {
			*plaintext_length = operation->unprocessed_input_bytes;
		}
	} else {
		/* In this case ciphertext and AD has length 0 and we don't have a context.
		 * Initialize it here, so we can verify the tag.
		 */
		initialize_or_resume_context(operation);
	}

	return finalize_aead_decryption(operation, tag);
#endif
}

psa_status_t cracen_aead_abort(cracen_aead_operation_t *operation)
{
	safe_memzero((void *)operation, sizeof(cracen_aead_operation_t));
	return PSA_SUCCESS;
}

static psa_status_t feed_singlepart_ccm_aad(cracen_aead_operation_t *operation,
					    const uint8_t *additional_data,
					    size_t additional_data_length)
{
	psa_status_t status;
	/* Data fed to CRACEN needs to remain untouched until it's been consumed
	 * (sx_aead_wait()), so don't put the CCM header buffer on the stack.
	 * This is not thread-safe but the Silex driver functions take care of
	 * locking and unlocking a mutex which ensures that there can be only
	 * one active caller at the same time.
	 */
	static uint8_t ccm_header_aad[ROUND_UP(CCM_HEADER_MAX_LENGTH,
					       PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES))];
	size_t ccm_header_length;
	size_t aad_fed_count;

	create_aead_ccmheader(operation, ccm_header_aad, &ccm_header_length);

	if (additional_data_length != 0) {
		/* Data fed to CRACEN needs to be block size-aligned, so
		 * complete the header with the beginning of the user-provided AAD.
		 */
		aad_fed_count = MIN(additional_data_length,
				    sizeof(ccm_header_aad) - ccm_header_length);
		memcpy(ccm_header_aad + ccm_header_length, additional_data, aad_fed_count);
	} else {
		aad_fed_count = 0;
	}

	status = cracen_feed_data_to_hw(operation, ccm_header_aad,
					ccm_header_length + aad_fed_count, NULL, true);
	if (status != PSA_SUCCESS) {
		return status;
	}

	if (additional_data_length != aad_fed_count) {
		/* Feed the rest of the user-provided AAD.
		 * The last feeding doesn't need to be block size-aligned.
		 */
		status = cracen_feed_data_to_hw(operation, additional_data + aad_fed_count,
						additional_data_length - aad_fed_count, NULL, true);
	}

	return status;
}

psa_status_t cracen_aead_encrypt(const psa_key_attributes_t *attributes, const uint8_t *key_buffer,
				 size_t key_buffer_size, psa_algorithm_t alg, const uint8_t *nonce,
				 size_t nonce_length, const uint8_t *additional_data,
				 size_t additional_data_length, const uint8_t *plaintext,
				 size_t plaintext_length, uint8_t *ciphertext,
				 size_t ciphertext_size, size_t *ciphertext_length)
{
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
	cracen_aead_operation_t operation = {0};
	size_t tag_length = 0;

	if (ciphertext_size < plaintext_length) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	status = setup(&operation, CRACEN_ENCRYPT, attributes, key_buffer, key_buffer_size, alg);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	set_lengths(&operation, additional_data_length, plaintext_length);

	/* Do not call the cracen_aead_update*() functions to avoid using
	 * HW context switching (process_on_hw()) in single-part operations.
	 */

	status = set_nonce(&operation, nonce, nonce_length);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	if (IS_ENABLED(PSA_NEED_CRACEN_CCM_AES) && operation.alg == PSA_ALG_CCM) {
		/* CCM has a header which is prepended to the additional data. */
		status = feed_singlepart_ccm_aad(&operation, additional_data,
						 additional_data_length);
	} else {
		status = cracen_feed_data_to_hw(&operation, additional_data,
						additional_data_length, NULL, true);
	}
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	status = cracen_feed_data_to_hw(&operation, plaintext, plaintext_length, ciphertext, false);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	status = finalize_aead_encryption(&operation, &ciphertext[plaintext_length],
					  ciphertext_size - plaintext_length, &tag_length);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	*ciphertext_length = plaintext_length + tag_length;
	return PSA_SUCCESS;

error_exit:
	*ciphertext_length = 0;
	cracen_aead_abort(&operation);
	return status;
}

psa_status_t cracen_aead_decrypt(const psa_key_attributes_t *attributes, const uint8_t *key_buffer,
				 size_t key_buffer_size, psa_algorithm_t alg, const uint8_t *nonce,
				 size_t nonce_length, const uint8_t *additional_data,
				 size_t additional_data_length, const uint8_t *ciphertext,
				 size_t ciphertext_length, uint8_t *plaintext,
				 size_t plaintext_size, size_t *plaintext_length)
{
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
	cracen_aead_operation_t operation = {0};

	status = setup(&operation, CRACEN_DECRYPT, attributes, key_buffer, key_buffer_size, alg);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	*plaintext_length = ciphertext_length - operation.tag_size;

	if (plaintext_size < *plaintext_length) {
		status = PSA_ERROR_BUFFER_TOO_SMALL;
		goto error_exit;
	}

	set_lengths(&operation, additional_data_length, *plaintext_length);

	/* Do not call the cracen_aead_update*() functions to avoid using
	 * HW context switching (process_on_hw()) in single-part operations.
	 */

	status = set_nonce(&operation, nonce, nonce_length);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	if (IS_ENABLED(PSA_NEED_CRACEN_CCM_AES) && operation.alg == PSA_ALG_CCM) {
		/* CCM has a header which is prepended to the additional data. */
		status = feed_singlepart_ccm_aad(&operation, additional_data,
						 additional_data_length);
	} else {
		status = cracen_feed_data_to_hw(&operation, additional_data,
						additional_data_length, NULL, true);
	}
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	status = cracen_feed_data_to_hw(&operation, ciphertext,
					*plaintext_length, plaintext, false);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	status = finalize_aead_decryption(&operation, &ciphertext[*plaintext_length]);
	if (status != PSA_SUCCESS) {
		goto error_exit;
	}

	return PSA_SUCCESS;

error_exit:
	*plaintext_length = 0;
	cracen_aead_abort(&operation);
	return status;
}
