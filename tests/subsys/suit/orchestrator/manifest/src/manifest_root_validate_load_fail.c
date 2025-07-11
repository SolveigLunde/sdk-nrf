/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <stddef.h>

/** @brief Valid SUIT envelope, based on ../root_validate_load_fail.yaml
 *
 */
const uint8_t manifest_root_validate_load_fail_buf[] = {
	0xD8, 0x6B, 0xA2, 0x02, 0x58, 0x7A, 0x82, 0x58, 0x24, 0x82, 0x2F, 0x58, 0x20, 0xE2, 0x9F,
	0x20, 0x0A, 0xA7, 0x4B, 0xD4, 0xAD, 0xCD, 0x76, 0xBD, 0xA9, 0x76, 0xFB, 0xB3, 0xD7, 0x54,
	0x46, 0xD2, 0x6A, 0xF7, 0xCE, 0x7D, 0x4F, 0x34, 0x26, 0x43, 0x40, 0x90, 0xAA, 0x39, 0x5B,
	0x58, 0x51, 0xD2, 0x84, 0x4A, 0xA2, 0x01, 0x26, 0x04, 0x45, 0x1A, 0x40, 0x00, 0x00, 0x00,
	0xA0, 0xF6, 0x58, 0x40, 0xE5, 0xED, 0xD6, 0xEA, 0x6B, 0x52, 0x91, 0xE3, 0xB2, 0x41, 0xCA,
	0x5B, 0xE8, 0x69, 0x10, 0x2C, 0x20, 0x16, 0x1B, 0xD5, 0x13, 0x20, 0x57, 0x2A, 0x96, 0xC6,
	0x68, 0x4E, 0x53, 0x12, 0x96, 0xFD, 0x15, 0x6C, 0xF3, 0xBB, 0x09, 0x9F, 0xCE, 0x61, 0xC7,
	0x23, 0x45, 0xF2, 0x13, 0x62, 0x38, 0x8C, 0xA0, 0x19, 0xE5, 0x67, 0x65, 0x66, 0xE4, 0xC8,
	0x73, 0x86, 0x26, 0x1A, 0x8D, 0x72, 0x27, 0xB0, 0x03, 0x58, 0x8C, 0xA7, 0x01, 0x01, 0x02,
	0x01, 0x03, 0x58, 0x52, 0xA3, 0x02, 0x81, 0x82, 0x4A, 0x69, 0x43, 0x41, 0x4E, 0x44, 0x5F,
	0x4D, 0x46, 0x53, 0x54, 0x41, 0x00, 0x04, 0x58, 0x3A, 0x82, 0x14, 0xA3, 0x01, 0x50, 0x76,
	0x17, 0xDA, 0xA5, 0x71, 0xFD, 0x5A, 0x85, 0x8F, 0x94, 0xE2, 0x8D, 0x73, 0x5C, 0xE9, 0xF4,
	0x02, 0x50, 0x97, 0x05, 0x48, 0x23, 0x4C, 0x3D, 0x59, 0xA1, 0x89, 0x86, 0xA5, 0x46, 0x60,
	0xA1, 0x4B, 0x0A, 0x18, 0x18, 0x50, 0x9C, 0x1B, 0x1E, 0x37, 0x2C, 0xB4, 0x5C, 0x33, 0x92,
	0xDD, 0x49, 0x56, 0x6B, 0x18, 0x31, 0x93, 0x01, 0xA1, 0x00, 0xA0, 0x07, 0x43, 0x82, 0x0C,
	0x00, 0x08, 0x46, 0x84, 0x0C, 0x00, 0x18, 0x18, 0x00, 0x09, 0x43, 0x82, 0x0C, 0x00, 0x05,
	0x82, 0x4C, 0x6B, 0x49, 0x4E, 0x53, 0x54, 0x4C, 0x44, 0x5F, 0x4D, 0x46, 0x53, 0x54, 0x50,
	0x97, 0x05, 0x48, 0x23, 0x4C, 0x3D, 0x59, 0xA1, 0x89, 0x86, 0xA5, 0x46, 0x60, 0xA1, 0x4B,
	0x0A};

const size_t manifest_root_validate_load_fail_len = sizeof(manifest_root_validate_load_fail_buf);
