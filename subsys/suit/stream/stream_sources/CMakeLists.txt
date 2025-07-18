#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_interface_library_named(suit_stream_sources_interface)
target_include_directories(suit_stream_sources_interface INTERFACE include)
target_link_libraries(suit_stream_sources_interface INTERFACE suit_platform_err)
target_link_libraries(suit_stream_sources_interface INTERFACE suit_stream_sinks_interface)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SOURCE_CACHE src/suit_dfu_cache_streamer.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SOURCE_MEMPTR src/suit_memptr_streamer.c)

zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_IPC_REQUESTOR src/suit_ipc_streamer_requestor.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_IPC_PROVIDER src/suit_ipc_streamer_provider.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_FETCH_SOURCE_MGR src/suit_fetch_source_mgr.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SOURCE_EXTMEM src/suit_extmem_streamer.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SOURCE_FLASH src/suit_flash_streamer.c)
zephyr_library_sources(src/suit_generic_address_streamer.c)

zephyr_library_link_libraries(suit_stream_sources_interface)
zephyr_library_link_libraries(suit_memory_layout_interface)
zephyr_library_link_libraries(suit_source_selector_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_SOURCE_CACHE suit_cache_interface)
