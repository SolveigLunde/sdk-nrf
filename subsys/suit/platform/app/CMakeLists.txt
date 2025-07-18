#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# SUIT APP Platform API
zephyr_interface_library_named(suit_app_platform)
target_include_directories(suit_app_platform INTERFACE include)
target_link_libraries(suit_app_platform INTERFACE suit)

zephyr_library()

zephyr_library_sources(src/suit_plat_swap.c)
zephyr_library_sources(src/suit_plat_fetch_app_specific.c)
zephyr_library_sources(src/suit_plat_copy_app_specific.c)
zephyr_library_sources(src/suit_plat_write_app_specific.c)
zephyr_library_sources(src/suit_plat_retrieve_manifest_app_specific.c)
zephyr_library_sources(src/suit_plat_version_app_specific.c)
zephyr_library_sources_ifdef(CONFIG_SUIT_CHECK_IMAGE_MATCH src/suit_plat_check_image_match_app_specific.c)
zephyr_library_sources(src/suit_plat_authenticate.c)
zephyr_library_sources(src/suit_plat_devconfig.c)
zephyr_library_sources(src/suit_plat_check_content_app_specific.c)
zephyr_library_sources(src/suit_plat_manifest_info.c)
zephyr_library_sources(src/suit_plat_invoke.c)

zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM suit_stream_sinks_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STREAM suit_stream_sources_interface)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_SINK_SELECTOR suit_sink_selector_interface)

zephyr_library_link_libraries(suit_utils)
zephyr_library_link_libraries(suit_platform_interface)
zephyr_library_link_libraries(suit_memptr_storage_interface)
