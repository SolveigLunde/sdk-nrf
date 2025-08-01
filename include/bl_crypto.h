/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BOOTLOADER_CRYPTO_H__
#define BOOTLOADER_CRYPTO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <fw_info.h>
#if defined(CONFIG_NRF_SECURITY)
#include <psa/crypto.h>
#include <psa/crypto_extra.h>
#endif

/** @defgroup bl_crypto Bootloader crypto functions
 * @{
 */

/* Placeholder defines. Values should be updated, if no existing errors can be
 * used instead. */
#define EHASHINV 101
#define ESIGINV  102

#if defined(CONFIG_SB_CRYPTO_PSA_SHA512)
typedef psa_hash_operation_t bl_sha512_ctx_t;
#elif defined(CONFIG_SB_CRYPTO_OBERON_SHA256)
#include <ocrypto_sha256.h>
#define SHA256_CTX_SIZE sizeof(ocrypto_sha256_ctx)
typedef ocrypto_sha256_ctx bl_sha256_ctx_t;
#elif defined(CONFIG_SB_CRYPTO_CC310_SHA256)
#include <nrf_cc310_bl_hash_sha256.h>
#define SHA256_CTX_SIZE sizeof(nrf_cc310_bl_hash_context_sha256_t)
typedef nrf_cc310_bl_hash_context_sha256_t bl_sha256_ctx_t;
#else
#define SHA256_CTX_SIZE 256
/* uint32_t to make sure it is aligned equally as the other contexts. */
typedef uint32_t bl_sha256_ctx_t[SHA256_CTX_SIZE/4];
#endif


/**
 * @brief Initialize bootloader crypto module.
 *
 * @retval 0        On success.
 * @retval -EFAULT  If crypto module reported an error.
 */
int bl_crypto_init(void);


/**
 * @brief Verify a signature using configured signature and SHA-256
 *
 * Verifies the public key against the public key hash, then verifies the hash
 * of the signed data against the signature using the public key.
 *
 * @param[in]  public_key       Public key.
 * @param[in]  public_key_hash  Expected hash of the public key. This is the
 *                              root of trust.
 * @param[in]  signature        Firmware signature.
 * @param[in]  firmware         Firmware.
 * @param[in]  firmware_len     Length of firmware.
 *
 * @retval 0          On success.
 * @retval -EHASHINV  If public_key_hash didn't match public_key.
 * @retval -ESIGINV   If signature validation failed.
 * @return Any error code from @ref bl_sha256_init, @ref bl_sha256_update,
 *         @ref bl_sha256_finalize, or @ref bl_secp256r1_validate if something
 *         else went wrong.
 *
 * @remark No parameter can be NULL.
 */
int bl_root_of_trust_verify(const uint8_t *public_key,
			    const uint8_t *public_key_hash,
			    const uint8_t *signature,
			    const uint8_t *firmware,
			    const uint32_t firmware_len);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_root_of_trust_verify_t)(
			    const uint8_t *public_key,
			    const uint8_t *public_key_hash,
			    const uint8_t *signature,
			    const uint8_t *firmware,
			    const uint32_t firmware_len);


/**
 * @brief Implementation of rot_verify that is safe to be called from EXT_API.
 *
 * See @ref bl_root_of_trust_verify for docs.
 */
int bl_root_of_trust_verify_external(const uint8_t *public_key,
				     const uint8_t *public_key_hash,
				     const uint8_t *signature,
				     const uint8_t *firmware,
				     const uint32_t firmware_len);

/**
 * @brief Initialize a sha256 operation context variable.
 *
 * @param[out]  ctx  Context to be initialized.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL.
 */
int bl_sha256_init(bl_sha256_ctx_t *ctx);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_sha256_init_t)(bl_sha256_ctx_t *ctx);


/**
 * @brief Hash a portion of data.
 *
 * @note @p ctx must be initialized before being used in this function.
 *       An uninitialized @p ctx might not be reported as an error. Also,
 *       @p ctx must not be used if it has been finalized, though this might
 *       also not be reported as an error.
 *
 * @param[in]  ctx       Context variable. Must have been initialized.
 * @param[in]  data      Data to hash.
 * @param[in]  data_len  Length of @p data.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL, uninitialized, or corrupted.
 * @retval -ENOSYS   If the context has already been finalized.
 */
int bl_sha256_update(bl_sha256_ctx_t *ctx, const uint8_t *data, uint32_t data_len);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_sha256_update_t)(bl_sha256_ctx_t *ctx, const uint8_t *data,
				uint32_t data_len);


/**
 * @brief Finalize a hash result.
 *
 * @param[in]  ctx       Context variable.
 * @param[out] output    Where to put the resulting digest. Must be at least
 *                       32 bytes long.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL or corrupted, or @p output was NULL.
 */
int bl_sha256_finalize(bl_sha256_ctx_t *ctx, uint8_t *output);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_sha256_finalize_t)(bl_sha256_ctx_t *ctx, uint8_t *output);


/**
 * @brief Calculate a digest and verify it directly.
 *
 * @param[in]  data      The data to hash.
 * @param[in]  data_len  The length of @p data.
 * @param[in]  expected  The expected digest over @p data.
 *
 * @retval 0          If the procedure succeeded and the resulting digest is
 *                    identical to @p expected.
 * @retval -EHASHINV  If the procedure succeeded, but the digests don't match.
 * @return Any error code from @ref bl_sha256_init, @ref bl_sha256_update, or
 *         @ref bl_sha256_finalize if something else went wrong.
 */
int bl_sha256_verify(const uint8_t *data, uint32_t data_len, const uint8_t *expected);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_sha256_verify_t)(const uint8_t *data, uint32_t data_len,
				const uint8_t *expected);

#if defined(CONFIG_SB_CRYPTO_PSA_SHA512)
/**
 * @brief Initialize a sha512 operation context variable.
 *
 * @param[out]  ctx  Context to be initialized.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL.
 */
int bl_sha512_init(bl_sha512_ctx_t *ctx);

/**
 * @brief Hash a portion of data.
 *
 * @note @p ctx must be initialized before being used in this function.
 *       An uninitialized @p ctx might not be reported as an error. Also,
 *       @p ctx must not be used if it has been finalized, though this might
 *       also not be reported as an error.
 *
 * @param[in]  ctx       Context variable. Must have been initialized.
 * @param[in]  data      Data to hash.
 * @param[in]  data_len  Length of @p data.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL, uninitialized, or corrupted.
 * @retval -ENOSYS   If the context has already been finalized.
 */
int bl_sha512_update(bl_sha512_ctx_t *ctx, const uint8_t *data, uint32_t data_len);

/**
 * @brief Finalize a hash result.
 *
 * @param[in]  ctx       Context variable.
 * @param[out] output    Where to put the resulting digest. Must be at least
 *                       32 bytes long.
 *
 * @retval 0         On success.
 * @retval -EINVAL   If @p ctx was NULL or corrupted, or @p output was NULL.
 */
int bl_sha512_finalize(bl_sha512_ctx_t *ctx, uint8_t *output);

/**
 * @brief Calculate a digest and verify it directly.
 *
 * @param[in]  data      The data to hash.
 * @param[in]  data_len  The length of @p data.
 * @param[in]  expected  The expected digest over @p data.
 *
 * @retval 0          If the procedure succeeded and the resulting digest is
 *                    identical to @p expected.
 * @retval -EHASHINV  If the procedure succeeded, but the digests don't match.
 * @return Any error code from @ref bl_512_init, @ref bl_sha512_update, or
 *         @ref bl_sha512_finalize if something else went wrong.
 */
int bl_sha512_verify(const uint8_t *data, uint32_t data_len, const uint8_t *expected);
#endif

/**
 * @brief Validate a secp256r1 signature.
 *
 * @param[in]  hash        The hash to validate against.
 * @param[in]  hash_len    The length of the hash.
 * @param[in]  public_key  The public key to validate with.
 * @param[in]  signature   The signature to validate.
 *
 * @retval 0         The operation succeeded and the signature is valid for the
 *                   hash.
 * @retval -EINVAL   A parameter was NULL, or the @p hash_len was not 32 bytes.
 * @retval -ESIGINV  The signature validation failed.
 */
int bl_secp256r1_validate(const uint8_t *hash,
			  uint32_t hash_len,
			  const uint8_t *public_key,
			  const uint8_t *signature);

/* Typedef for use in EXT_API declaration */
typedef int (*bl_secp256r1_validate_t)(
			  const uint8_t *hash,
			  uint32_t hash_len,
			  const uint8_t *public_key,
			  const uint8_t *signature);

/**
 * @brief Validate an ed25519 signature.
 *
 * @param[in]  hash        The hash to validate against.
 * @param[in]  hash_len    The length of the hash.
 * @param[in]  signature   The signature to validate.
 *
 * @retval 0         The operation succeeded and the signature is valid for the
 *                   hash.
 * @retval -EINVAL   A parameter was NULL, or the @p hash_len was not 64 bytes.
 * @retval -ESIGINV  The signature validation failed.
 */
int bl_ed25519_validate(const uint8_t *hash,
			uint32_t hash_len,
			const uint8_t *signature);

/**
 * @brief Structure describing the BL_ROT_VERIFY EXT_API.
 */
struct bl_rot_verify_ext_api {
	bl_root_of_trust_verify_t bl_root_of_trust_verify;
};

/**
 * @brief Structure describing the BL_SHA256 EXT_API.
 */
struct bl_sha256_ext_api {
	bl_sha256_init_t bl_sha256_init;
	bl_sha256_update_t bl_sha256_update;
	bl_sha256_finalize_t bl_sha256_finalize;
	bl_sha256_verify_t bl_sha256_verify;
	uint32_t bl_sha256_ctx_size;
};

/**
 * @brief Structure describing the BL_SECP256R1 EXT_API.
 */
struct bl_secp256r1_ext_api {
	bl_secp256r1_validate_t bl_secp256r1_validate;
};

  /** @} */

#ifdef __cplusplus
}
#endif

#endif
