/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef FLASH_STREAMER_H__
#define FLASH_STREAMER_H__

#include <suit_sink.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check if address can be streamed from flash memory.
 *
 * @param address Pointer to the streamable payload
 *
 * @return True if address can be streamed from flash memory, false otherwise.
 */
bool suit_flash_streamer_address_in_range(const uint8_t *address);

/**
 * @brief Stream payload from pointer to sink
 *
 * @param payload Pointer to the streamable payload
 * @param payload_size Size of the payload
 * @param sink Pointer to sink to write the data to
 *
 * @return SUIT_PLAT_SUCCESS if success otherwise error code
 */
suit_plat_err_t suit_flash_streamer_stream(const uint8_t *payload, size_t payload_size,
					   struct stream_sink *sink);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STREAMER_H__ */
