#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_interface_library_named(suit_cache_interface)
target_include_directories(suit_cache_interface INTERFACE include)
target_link_libraries(suit_cache_interface INTERFACE suit_platform_err)

zephyr_library()
zephyr_library_sources(src/suit_dfu_cache.c)
zephyr_library_sources(src/suit_dfu_cache_helpers.c)

zephyr_library_sources_ifdef(CONFIG_SUIT_CACHE_RW src/suit_dfu_cache_rw.c)

zephyr_library_link_libraries(suit_cache_interface)

zephyr_library_link_libraries_ifdef(CONFIG_SUIT_CACHE_RW suit_utils)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_CACHE_RW suit_envelope_info)

zephyr_library_link_libraries(suit_stream_sources_interface)
zephyr_library_link_libraries(suit_source_selector_interface)
zephyr_library_link_libraries(suit_memory_layout_interface)
zephyr_library_link_libraries(suit_stream_sinks_interface)
