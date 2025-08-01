/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Lib KMU header.
 */

#ifndef __LIB_KMU_H
#define __LIB_KMU_H

/**
 * @defgroup lib_kmu Lib Key Management Unit
 * @brief Lib KMU
 *
 * @{
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DLT-4257 */
#if defined(CONFIG_SOC_NRF54LM20A) || defined(CONFIG_SOC_NRF54LV10A)
#undef CRACEN_PROTECTED_RAM_AES_KEY0
#undef CRACEN_PROTECTED_RAM_AES_KEY1

#if defined(CONFIG_SOC_NRF54LM20A)
#define CRACEN_PROTECTED_RAM_AES_KEY0 0x2007FEE0
#define CRACEN_PROTECTED_RAM_AES_KEY1 0x2007FF00
#elif defined(CONFIG_SOC_NRF54LV10A)
#define CRACEN_PROTECTED_RAM_AES_KEY0 0x2002FEE0
#define CRACEN_PROTECTED_RAM_AES_KEY1 0x2002FF00
#endif

#endif /* CONFIG_SOC_NRF54LM20A || CONFIG_SOC_NRF54LV10A */

/** @def LIB_KMU_SUCCESS
 *
 * @brief Indicates a successful KMU operation.
 */
#define LIB_KMU_SUCCESS	 0x0
/** @def LIB_KMU_ERROR
 *
 * @brief Error during the KMU operation.
 */
#define LIB_KMU_ERROR	 0x1
/** @def LIB_KMU_NULL_PNT
 *
 * @brief Got a null pointer for KMU data.
 */
#define LIB_KMU_NULL_PNT 0x2
/** @def LIB_KMU_REVOKED
 *
 * @brief The key is revoked.
 */
#define LIB_KMU_REVOKED	 0x3

/** @brief KMU revocation policies.
 */
enum lib_kmu_rev_policy {
	LIB_KMU_REV_POLICY_RESERVED,
	LIB_KMU_REV_POLICY_ROTATING,
	LIB_KMU_REV_POLICY_LOCKED,
	LIB_KMU_REV_POLICY_REVOKED
};

/**
 * @brief Source struct for KMU slot provisioning.
 */
struct kmu_src {
	/** Asset contents/value. */
	uint32_t value[4];
	/** Revocation policy. */
	uint32_t rpolicy;
	/** 32-bit destination address. Cannot point to SICR and must be on a
	 * 128-bit boundary.
	 */
	uint32_t dest;
	/** 32 bits of any clear-text metadata that belongs with the key slot.
	 */
	uint32_t metadata;
};

/** @brief Provision a KMU slot with data.
 *
 *  @param[in] slot_id Slot id of the KMU slot.
 *  @param[in] kmu_src Pointer to the KMU source data to be provisioned.
 *
 *  @return LIB_KMU_SUCCESS        If the operation was successful.
 *  @return -LIB_KMU_ERROR         If the operation returned an error.
 *  @return -LIB_KMU_NULL_POINTER  If the KMU source is NULL.
 */
int lib_kmu_provision_slot(int slot_id, struct kmu_src *kmu_src);

/** @brief Push the KMU slot to its destination address.
 *
 *  @param[in] slot_id Slot id of the KMU slot.
 *
 *  @return LIB_KMU_SUCCESS        If the operation was successful.
 *  @return -LIB_KMU_ERROR         If the operation returned an error.
 */
int lib_kmu_push_slot(int slot_id);

/** @brief Block one or more consecutive KMU slots.
 *
 *  @param[in] slot_id ID of the first KMU slot to block.
 *  @param[in] slot_count Number of consecutive slots to block.
 *
 *  @return LIB_KMU_SUCCESS        If the operation was successful.
 *  @return -LIB_KMU_ERROR         If the operation returned an error.
 *
 *  @note This uses the `PUSHBLOCK` task on devices that do not have the `BLOCK` one.
 */
int lib_kmu_block_slot_range(int slot_id, unsigned int slot_count);

/** @brief Revoke the KMU slot.
 *
 *  @param[in] slot_id Slot id of the KMU slot.
 *
 *  @return LIB_KMU_SUCCESS        If the operation was successful.
 *  @return -LIB_KMU_ERROR         If the operation returned an error.
 */
int lib_kmu_revoke_slot(int slot_id);

/** @brief Read the metadata of a KMU slot.
 *
 *  @param[in] slot_id Slot id of the KMU slot.
 *  @param[out] metadata Ptr to where the meta data will be written to.
 *
 *  @return LIB_KMU_SUCCESS        If the operation was successful.
 *  @return -LIB_KMU_ERROR         If the operation returned an error.
 *  @return -LIB_KMU_NULL_POINTER  If the metadata is NULL.
 */
int lib_kmu_read_metadata(int slot_id, uint32_t *metadata);

/** @brief Check if a KMU slot is empty.
 *
 *  @param[in] slot_id Slot id of the KMU slot.
 *
 *  @return true  If the KMU slot is empty.
 *  @return false If the KMU slot is not empty.
 */
bool lib_kmu_is_slot_empty(int slot_id);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* LIB_KMU */
