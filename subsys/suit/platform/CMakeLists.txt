#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# SUIT Platform API
zephyr_interface_library_named(suit_platform_interface)
target_include_directories(suit_platform_interface INTERFACE include)

# SUIT Platform implementation
zephyr_library()

zephyr_library_sources(src/suit_plat_components.c)
zephyr_library_sources(src/suit_plat_commands.c)
zephyr_library_sources(src/suit_plat_version.c)
zephyr_library_sources(src/suit_plat_fetch.c)
zephyr_library_sources(src/suit_plat_retrieve_manifest.c)
zephyr_library_sources(src/suit_plat_copy.c)
zephyr_library_sources(src/suit_plat_write.c)
zephyr_library_sources(src/suit_plat_check_content.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_CHECK_IMAGE_MATCH src/suit_plat_check_image_match.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_MEMPTR_STORAGE src/suit_plat_memptr_size_update.c)
zephyr_library_sources(src/suit_plat_error_convert.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_DIGEST src/suit_plat_digest.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_PLAT_CHECK_CLASSES src/suit_plat_class_check.c)

zephyr_library_link_libraries(suit)
zephyr_library_link_libraries(suit_platform_err)
zephyr_library_link_libraries(suit_utils)
zephyr_library_link_libraries(suit_platform_interface)
zephyr_library_link_libraries(suit_memory_layout_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM suit_stream_sinks_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM suit_stream_sources_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_MEMPTR_STORAGE suit_memptr_storage_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_SINK_SELECTOR suit_sink_selector_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_FILTER_DECRYPT suit_stream_filters_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM_FILTER_DECOMPRESS suit_stream_filters_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_MANIFEST_VARIABLES suit_manifest_variables)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_IPUC suit_ipuc)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STORAGE suit_storage_interface)

if(CONFIG_SUIT_PLATFORM_VARIANT_APP)
	add_subdirectory(app)
elseif(CONFIG_SUIT_PLATFORM_VARIANT_SDFW)
	add_subdirectory(sdfw)
else()
  message(FATAL_ERROR "Please select one of the supported SUIT platform variants")
endif()

add_subdirectory_ifdef(CONFIG_SUIT_SINK_SELECTOR sink_selector)

# SUIT common library definition (backward compatibility)
zephyr_interface_library_named(suit_common)
