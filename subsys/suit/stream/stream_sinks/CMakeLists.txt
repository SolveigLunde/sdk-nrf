#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_interface_library_named(suit_stream_sinks_interface)
target_include_directories(suit_stream_sinks_interface INTERFACE include)
if(CONFIG_SUIT_STREAM_SINK_MEMPTR OR CONFIG_SUIT_STREAM_SINK_FLASH)
target_link_libraries(suit_stream_sinks_interface INTERFACE suit_memptr_storage_interface)
endif() # CONFIG_SUIT_STREAM_SINK_MEMPTR
target_link_libraries(suit_stream_sinks_interface INTERFACE suit_platform_err)



zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_CACHE src/suit_dfu_cache_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_RAM src/suit_ram_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_FLASH src/suit_flash_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_MEMPTR src/suit_memptr_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_SDFW src/suit_sdfw_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_SDFW_RECOVERY src/suit_sdfw_recovery_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_DIGEST src/suit_digest_sink.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_STREAM_SINK_EXTMEM src/suit_extmem_sink.c)

zephyr_library_link_libraries(suit_stream_sinks_interface)
zephyr_library_link_libraries(suit_memory_layout_interface)

zephyr_library_link_libraries_ifdef(CONFIG_SUIT_SINK_SELECTOR suit_sink_selector_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_SINK_CACHE suit_cache_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_SINK_SDFW suit_utils)
