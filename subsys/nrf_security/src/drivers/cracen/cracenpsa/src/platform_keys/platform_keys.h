/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef CRACEN_PLATFORM_KEYS_H
#define CRACEN_PLATFORM_KEYS_H

#include <cracen_psa.h>

psa_status_t cracen_platform_get_builtin_key(psa_drv_slot_number_t slot_number,
					     psa_key_attributes_t *attributes, uint8_t *key_buffer,
					     size_t key_buffer_size, size_t *key_buffer_length);

psa_status_t cracen_platform_keys_get_size(psa_key_attributes_t const *attributes,
					   size_t *key_size);

bool cracen_platform_keys_is_ikg_key(psa_key_attributes_t const *attributes);

uint32_t cracen_platform_keys_get_owner(psa_key_attributes_t const *attributes);

psa_status_t cracen_platform_get_key_slot(mbedtls_svc_key_id_t key_id, psa_key_lifetime_t *lifetime,
					  psa_drv_slot_number_t *slot_number);

psa_status_t cracen_platform_keys_provision(const psa_key_attributes_t *attributes,
					    const uint8_t *key_buffer, size_t key_buffer_size);

psa_status_t cracen_platform_destroy_key(const psa_key_attributes_t *attributes);

#endif /* CRACEN_PLATFORM_KEYS_H */
