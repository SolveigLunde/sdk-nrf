/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <psa/crypto.h>
#include <suit_platform.h>
#include <suit_mci.h>
#include <suit_plat_decode_util.h>
#include <suit_plat_component_compatibility.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_SUIT_MANIFEST_VARIABLES
#include <suit_manifest_variables.h>
#include <suit_storage_mpi.h>
#endif /* CONFIG_SUIT_MANIFEST_VARIABLES */

LOG_MODULE_REGISTER(suit_plat_authenticate, CONFIG_SUIT_LOG_LEVEL);

int suit_plat_authenticate_manifest(struct zcbor_string *manifest_component_id,
				    enum suit_cose_alg alg_id, struct zcbor_string *key_id,
				    struct zcbor_string *signature, struct zcbor_string *data)
{
	psa_algorithm_t psa_alg;
	uint32_t public_key_id = 0;
	suit_manifest_class_id_t *class_id = NULL;

	switch (alg_id) {
	case suit_cose_es256:
		psa_alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
		break;
	case suit_cose_EdDSA:
		psa_alg = PSA_ALG_PURE_EDDSA; /* ed25519/curve25519 without internal hashing */
		break;
	case suit_cose_VS_HashEdDSA:
		psa_alg = PSA_ALG_ED25519PH; /* ed25519/curve25519 with internal hashing */
		break;
	default:
		return SUIT_ERR_DECODING;
	}

	if ((manifest_component_id == NULL) || (key_id == NULL) || (signature == NULL) ||
	    (data == NULL) || (key_id->value == NULL) || (key_id->len == 0) ||
	    (signature->value == NULL) || (signature->len == 0) || (data->value == NULL) ||
	    (data->len == 0)) {
		return SUIT_ERR_DECODING;
	}

	/* Check if signature verification should be skipped. */
	int err = suit_plat_authorize_unsigned_manifest(manifest_component_id);

	if (err == SUIT_SUCCESS) {
		LOG_WRN("Signature verification skipped due to MCI configuration.");
		return err;
	}

	/* Check if component ID is a manifest class */
	if (suit_plat_decode_manifest_class_id(manifest_component_id, &class_id) !=
	    SUIT_PLAT_SUCCESS) {
		LOG_ERR("Component ID is not a manifest class");
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	/* Validate manifest class ID against supported manifests */
	mci_err_t ret = suit_mci_manifest_class_id_validate(class_id);

	if (ret != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Manifest class ID validation failed: MCI err %i", ret);
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	/* Try to get uint32_t key_id from zcbor_string */
	if (suit_plat_decode_key_id(key_id, &public_key_id) != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Decoding key ID failed");
		return SUIT_ERR_UNSUPPORTED_PARAMETER;
	}

	/* Validate KEY ID */
	ret = suit_mci_signing_key_id_and_alg_validate(class_id, public_key_id, alg_id);
	if (ret != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Signing key validation failed: MCI err %i", ret);
		return SUIT_ERR_AUTHENTICATION;
	}

#ifdef CONFIG_ZTEST
	ret = suit_mci_signing_key_id_get(class_id, &public_key_id);
	if (ret != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Unable to find volatile key ID: MCI err %i", ret);
		return SUIT_ERR_AUTHENTICATION;
	}
#endif /* CONFIG_ZTEST */

#ifdef MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER
	mbedtls_svc_key_id_t key;

	key.MBEDTLS_PRIVATE(key_id) = public_key_id;
	key.MBEDTLS_PRIVATE(owner) = NRF_OWNER_SECURE;
#else  /* MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER */
	psa_key_id_t key = public_key_id;
#endif /* MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER */

	/* Verify data */
	if (psa_verify_message(key, psa_alg, data->value, data->len, signature->value,
			       signature->len) == PSA_SUCCESS) {
		return SUIT_SUCCESS;
	}

	LOG_ERR("Signature verification failed.");

	return SUIT_ERR_AUTHENTICATION;
}

int suit_plat_authorize_unsigned_manifest(struct zcbor_string *manifest_component_id)
{
	suit_manifest_class_id_t *class_id = NULL;

	if ((manifest_component_id == NULL) || (manifest_component_id->value == NULL) ||
	    (manifest_component_id->len == 0)) {
		return SUIT_ERR_DECODING;
	}

	/* Check if component ID is a manifest class */
	if (suit_plat_decode_manifest_class_id(manifest_component_id, &class_id) !=
	    SUIT_PLAT_SUCCESS) {
		LOG_ERR("Component ID is not a manifest class");
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	/* Validate manifest class ID against supported manifests */
	mci_err_t ret = suit_mci_manifest_class_id_validate(class_id);

	if (ret != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Manifest class ID validation failed: MCI err %i", ret);
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	/* Check if unsigned manifest is allowed - pass key_id == 0*/
	ret = suit_mci_signing_key_id_and_alg_validate(class_id, 0, 0);

	if (ret == SUIT_PLAT_SUCCESS) {
		return SUIT_SUCCESS;
	}

	LOG_INF("Signature verification required.");

	return SUIT_ERR_AUTHENTICATION;
}

int suit_plat_authorize_component_id(struct zcbor_string *manifest_component_id,
				     struct zcbor_string *component_id)
{
	suit_manifest_class_id_t *class_id = NULL;

	if ((manifest_component_id == NULL) || (component_id == NULL) ||
	    (manifest_component_id->value == NULL) || (manifest_component_id->len == 0) ||
	    (component_id->value == NULL) || (component_id->len == 0)) {
		return SUIT_ERR_DECODING;
	}

	/* Check if component ID is a manifest class */
	if (suit_plat_decode_manifest_class_id(manifest_component_id, &class_id) !=
	    SUIT_PLAT_SUCCESS) {
		LOG_ERR("Component ID is not a manifest class");
		return SUIT_ERR_UNAUTHORIZED_COMPONENT;
	}

	return suit_plat_component_compatibility_check(class_id, component_id);
}

int suit_plat_authorize_process_dependency(struct zcbor_string *parent_component_id,
					   struct zcbor_string *child_component_id,
					   enum suit_command_sequence seq_name)
{
	suit_manifest_class_id_t *parent_class_id = NULL;
	suit_manifest_class_id_t *child_class_id = NULL;

	suit_plat_err_t err =
		suit_plat_decode_manifest_class_id(parent_component_id, &parent_class_id);
	if (err != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Unable to parse parent manifest class ID (err: %i)", err);
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	err = suit_plat_decode_manifest_class_id(child_component_id, &child_class_id);
	if (err != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Unable to parse child manifest class ID (err: %i)", err);
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	mci_err_t ret =
		suit_mci_manifest_process_dependency_validate(parent_class_id, child_class_id);
	if (ret == SUIT_PLAT_SUCCESS) {
		return SUIT_SUCCESS;
	}

	LOG_INF("Manifest dependency link unauthorized for sequence %d (err: %i)", seq_name, ret);

	return SUIT_ERR_UNAUTHORIZED_COMPONENT;
}

int suit_plat_authorize_var_rw_access(struct zcbor_string *manifest_component_id, uint32_t id)
{
#ifdef CONFIG_SUIT_MANIFEST_VARIABLES
	suit_manifest_role_t role = SUIT_MANIFEST_UNKNOWN;
	uint32_t required_access_bits = 0xFF;
	suit_manifest_class_id_t *class_id = NULL;
	suit_plat_err_t plat_ret;
	uint32_t access_mask;

	if ((manifest_component_id == NULL) || (manifest_component_id->value == NULL) ||
	    (manifest_component_id->len == 0)) {
		return SUIT_ERR_DECODING;
	}

	/* Check if component ID is a manifest class */
	if (suit_plat_decode_manifest_class_id(manifest_component_id, &class_id) !=
	    SUIT_PLAT_SUCCESS) {
		LOG_ERR("Component ID is not a manifest class");
		return SUIT_ERR_UNAUTHORIZED_COMPONENT;
	}

	if (suit_storage_mpi_role_get(class_id, &role) != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Failed to identify manifest role");
		return SUIT_ERR_UNAUTHORIZED_COMPONENT;
	}

	plat_ret = suit_mfst_var_get_access_mask(id, &access_mask);
	if (plat_ret != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Unsupported manifest variable %d: %d", id, plat_ret);
		return SUIT_ERR_UNAUTHORIZED_COMPONENT;
	}

	switch (role) {
	case SUIT_MANIFEST_APP_ROOT:
	case SUIT_MANIFEST_APP_RECOVERY:
	case SUIT_MANIFEST_APP_LOCAL_1:
	case SUIT_MANIFEST_APP_LOCAL_2:
	case SUIT_MANIFEST_APP_LOCAL_3:
		required_access_bits = MFST_VAR_ACCESS_APP;
		break;
	case SUIT_MANIFEST_RAD_RECOVERY:
	case SUIT_MANIFEST_RAD_LOCAL_1:
	case SUIT_MANIFEST_RAD_LOCAL_2:
		required_access_bits = MFST_VAR_ACCESS_RAD;
		break;
	case SUIT_MANIFEST_SEC_TOP:
	case SUIT_MANIFEST_SEC_SDFW:
	case SUIT_MANIFEST_SEC_SYSCTRL:
		required_access_bits = MFST_VAR_ACCESS_SEC;
		break;
	default:
		LOG_ERR("Unsupported manifest role: %d", role);
		return SUIT_ERR_UNAUTHORIZED_COMPONENT;
	}

	if ((access_mask & required_access_bits) == required_access_bits) {
		return SUIT_SUCCESS;
	}

#endif /* CONFIG_SUIT_MANIFEST_VARIABLES */
	return SUIT_ERR_UNAUTHORIZED_COMPONENT;
}
