#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_interface_library_named(suit_stream_filters_interface)
target_include_directories(suit_stream_filters_interface INTERFACE include)
target_link_libraries(suit_stream_filters_interface INTERFACE suit_platform_err)
target_link_libraries(suit_stream_filters_interface INTERFACE suit_stream_sinks_interface)
target_link_libraries(suit_stream_filters_interface INTERFACE suit)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_FILTER_DECRYPT src/suit_decrypt_filter.c)
# TODO: use AES KW unwrap algorithm from PSA API
zephyr_library_sources_ifdef(CONFIG_SUIT_AES_KW_MANUAL src/suit_aes_key_unwrap_manual.c)

zephyr_library_include_directories_ifdef(CONFIG_SUIT_STREAM_FILTER_DECOMPRESS ${NRF_DIR}/subsys/nrf_compress/lzma)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_FILTER_DECOMPRESS src/suit_decompress_filter.c)

zephyr_library_link_libraries(suit_stream_filters_interface)
zephyr_library_link_libraries(suit_utils)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_FILTER_DECRYPT suit_mci)
