/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef CRACEN_PSA_RSA_SIGNATURE_PSS_H
#define CRACEN_PSA_RSA_SIGNATURE_PSS_H

#include <cracen_psa_primitives.h>

int cracen_rsa_pss_sign_message(struct cracen_rsa_key *rsa_key, struct cracen_signature *signature,
				const struct sxhashalg *hashalg, const uint8_t *message,
				size_t message_length, size_t saltsz);

int cracen_rsa_pss_sign_digest(struct cracen_rsa_key *rsa_key, struct cracen_signature *signature,
			       const struct sxhashalg *hashalg, const uint8_t *input,
			       size_t input_length, size_t saltsz);

int cracen_rsa_pss_verify_message(struct cracen_rsa_key *rsa_key,
				  struct cracen_signature *signature,
				  const struct sxhashalg *hashalg, const uint8_t *message,
				  size_t message_length, size_t saltsz);

int cracen_rsa_pss_verify_digest(struct cracen_rsa_key *rsa_key, struct cracen_signature *signature,
				 const struct sxhashalg *hashalg, const uint8_t *digest,
				 size_t digest_length, size_t saltsz);

#endif /* CRACEN_PSA_RSA_SIGNATURE_PSS_H */
