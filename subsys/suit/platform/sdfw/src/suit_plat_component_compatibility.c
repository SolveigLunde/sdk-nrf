/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <suit_mci.h>
#include <suit_plat_decode_util.h>
#include <suit_platform_internal.h>
#include <suit_storage_mpi.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_SUIT_MANIFEST_VARIABLES
#include <suit_manifest_variables.h>
#endif /* CONFIG_SUIT_MANIFEST_VARIABLES */

/* -1 indicates no boot capability for given cpu id */
#define NO_BOOT_CAPABILITY_CPU_ID 255

LOG_MODULE_REGISTER(suit_plat_component_compat, CONFIG_SUIT_LOG_LEVEL);

int suit_plat_component_compatibility_check(const suit_manifest_class_id_t *class_id,
					    struct zcbor_string *component_id)
{
	suit_manifest_role_t role = SUIT_MANIFEST_UNKNOWN;
	suit_manifest_class_id_t *decoded_class_id;
	suit_component_type_t type = SUIT_COMPONENT_TYPE_UNSUPPORTED;
	intptr_t address;
	uint32_t number;
	uint8_t cpu_id;
	size_t size;

	if ((class_id == NULL) || (component_id == NULL) || (component_id->value == NULL) ||
	    (component_id->len == 0)) {
		return SUIT_ERR_DECODING;
	}

	/* Validate manifest class ID against supported manifests */
	mci_err_t ret = suit_mci_manifest_class_id_validate(class_id);

	if (ret != SUIT_PLAT_SUCCESS) {
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	if (suit_plat_decode_component_type(component_id, &type) != SUIT_PLAT_SUCCESS) {
		LOG_ERR("Unrecognized component type");
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	if (suit_storage_mpi_role_get(class_id, &role) != SUIT_PLAT_SUCCESS) {
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	switch (type) {
	case SUIT_COMPONENT_TYPE_MEM:
		/* Decode component_id */
		if (suit_plat_decode_component_id(component_id, &cpu_id, &address, &size) !=
		    SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		if (NO_BOOT_CAPABILITY_CPU_ID != cpu_id) {
			ret = suit_mci_processor_start_rights_validate(class_id, cpu_id);
			if (ret != SUIT_PLAT_SUCCESS) {
				return SUIT_ERR_UNAUTHORIZED_COMPONENT;
			}
		}

		ret = suit_mci_memory_access_rights_validate(class_id, (void *)address, size);
		if (ret != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}
		break;

	case SUIT_COMPONENT_TYPE_SOC_SPEC:
		if (suit_plat_decode_component_number(component_id, &number) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_DECODING;
		}

		ret = suit_mci_platform_specific_component_rights_validate(class_id, number);
		if (ret != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}
		break;

	case SUIT_COMPONENT_TYPE_CAND_MFST:
		if (suit_plat_decode_component_number(component_id, &number) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		if ((role != SUIT_MANIFEST_SEC_TOP) && (role != SUIT_MANIFEST_APP_ROOT) &&
		    (role != SUIT_MANIFEST_APP_RECOVERY)) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}
		break;

	case SUIT_COMPONENT_TYPE_CAND_IMG:
		if (suit_plat_decode_component_number(component_id, &number) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		if ((role == SUIT_MANIFEST_SEC_TOP) || (role == SUIT_MANIFEST_APP_ROOT)) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}
		break;

	case SUIT_COMPONENT_TYPE_CACHE_POOL:
		if (suit_plat_decode_component_number(component_id, &number) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		if ((role == SUIT_MANIFEST_SEC_TOP) || (role == SUIT_MANIFEST_APP_ROOT)) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}
		break;

	case SUIT_COMPONENT_TYPE_INSTLD_MFST:
		/* Decode manifest class id */
		if (suit_plat_decode_manifest_class_id(component_id, &decoded_class_id) !=
		    SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		/* Validate parent-child declarative relationship */
		ret = suit_mci_manifest_parent_child_declaration_validate(class_id,
									  decoded_class_id);
		if (ret != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNAUTHORIZED_COMPONENT;
		}

		break;

#ifdef CONFIG_SUIT_MANIFEST_VARIABLES

	case SUIT_COMPONENT_TYPE_MFST_VAR:

		uint32_t val;

		if (suit_plat_decode_component_number(component_id, &number) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		/* Let's check if given component is supported by reading out its content
		 */
		if (suit_mfst_var_get(number, &val) != SUIT_PLAT_SUCCESS) {
			return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
		}

		break;

#endif /* CONFIG_SUIT_MANIFEST_VARIABLES */

	default:
		return SUIT_ERR_UNSUPPORTED_COMPONENT_ID;
	}

	return SUIT_SUCCESS;
}
