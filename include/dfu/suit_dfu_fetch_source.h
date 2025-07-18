/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SUIT_DFU_FETCH_SOURCE_H__
#define SUIT_DFU_FETCH_SOURCE_H__

/** @file suit_dfu_fetch_source.h
 *
 * @brief The fetch source manager allows for registration of multiple fetch sources to be used by
 *        the SUIT processor during manifest processing.
 *
 * @details
 * A fetch source is any module, that registers it's URI handler through
 * the @ref suit_dfu_fetch_source_register and provides downloaded payload chunks using
 * the @ref suit_dfu_fetch_source_write_fetched_data API.
 * The SUIT proccessor iterates over the registered fetch sources, calling respective
 * @ref suit_dfu_fetch_source_request_fn.
 *
 * @ingroup suit
 * @{
 */

#if defined(CONFIG_SUIT_STREAM_FETCH_SOURCE_MGR) || defined(__DOXYGEN__)

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fetch source callback function.
 *
 * @details
 * Fetch source callbacks are registered via @ref suit_dfu_fetch_source_register.
 * Multiple such callbacks can be registered. The callbacks are called from inside
 * the SUIT processor and should be blocking until the download process finishes.
 * If a fetch source succeeds - the fetch operation is assumed to be completed
 * successfully, the iteration over fetch sources stops and the processor continues
 * processing the manifest sequence.
 * If a fetch source fails and it did not provide data using
 * @ref suit_dfu_fetch_source_write_fetched_data or @ref suit_dfu_fetch_source_seek
 * APIs, the iteration over fetch sources continues, and the next callback is called.
 * If the fetch source fails after providing data, the iteration over fetch sources stops
 * and the result code is passed to the SUIT processor to handle the error code.
 *
 * The role of the callback is to fetch data from the provided URI and pass it to SUIT
 * via @ref suit_dfu_fetch_source_write_fetched_data - the data does not have to be passed
 * all at once, @ref suit_dfu_fetch_source_write_fetched_data can be called multiple times
 * as new chunks of data arrive.
 * If the data needs to be written at some offset (e.g. if the chunks arrive out of order)
 * @ref suit_dfu_fetch_source_seek can be used to move the write pointer to the given offset.
 *
 * It is up to the implementer of this callback how the URI is resolved and how
 * the data is fetched.
 * If the function fails to fetch the data from the given URI it must return a negative
 * error code.
 *
 * @note The local fetch context should be bound to the session ID. It is possible that the
 *       fetch source will perform multiple downloads simultaneously.
 *
 * @param[in] uri         URI from which the data should be fetched. Fetch source interprets it.
 *                        In case if fetch source is unable to retrieve requested resource,
 *                        i.e. due to unsupported protocol, it shall just fail, without any prior
 *                        call to @ref suit_dfu_fetch_source_write_fetched_data or
 *                        @ref suit_dfu_fetch_source_seek.
 *
 * @param[in] uri_length  Length of the URI, in bytes.
 *
 * @param[in] session_id  Session ID used by the fetch source manager. It shall be passed without
 *                        modification to calls to @ref suit_dfu_fetch_source_write_fetched_data
 *                        and @ref suit_dfu_fetch_source_seek made by this function.
 *
 * @return 0 on success - if fetching from the URI is possible and writing the data succeeded
 *         Negative error code otherwise (URI not found or other error).
 */
typedef int (*suit_dfu_fetch_source_request_fn)(const uint8_t *uri, size_t uri_length,
						uint32_t session_id);

/**
 * @brief Register a fetch source
 *
 * @param[in] request_fn  Fetch source callback function. The suit processor will iterate over
 *                        registered callback functions when fetching based on a URI is needed.
 *
 * @warning Order of execution of callback functions is not guaranteed.
 *
 * @return 0 on success, negative error code otherwise.
 */
int suit_dfu_fetch_source_register(suit_dfu_fetch_source_request_fn request_fn);

/**
 * @brief Passes the fetched data to the SUIT processor.
 *
 * @details
 * This function shall only be called from inside a @ref suit_dfu_fetch_source_request_fn callback.
 *
 * @param[in] session_id  Session id, the same which was passed to the session_id parameter of the
 *                        @ref suit_dfu_fetch_source_request_fn callback.
 * @param[in] data        Pointer to the data to write.
 * @param[in] len         Length of the data to write.

 * @return 0 on success, negative error code otherwise.
 */
int suit_dfu_fetch_source_write_fetched_data(uint32_t session_id, const uint8_t *data, size_t len);

/**
 * @brief Move the internal write pointer inside the SUIT processor to the given offset.
 *
 * @details
 * This function shall only be called from inside a @ref suit_dfu_fetch_source_request_fn callback.
 *
 * @param[in] session_id  Session id, the same which was passed to the session_id parameter of the
 *                        @ref suit_dfu_fetch_source_request_fn callback.
 * @param[in] offset      The offset to which the write pointer shall be used.

 * @return 0 on success, negative error code otherwise.
 */
int suit_dfu_fetch_source_seek(uint32_t session_id, size_t offset);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SUIT_STREAM_FETCH_SOURCE_MGR || __DOXYGEN__ */

/**
 * @}
 */

#endif /* SUIT_DFU_FETCH_SOURCE_H__ */
