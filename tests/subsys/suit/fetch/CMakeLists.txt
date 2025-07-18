#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(integration_test_fetch)
include(../cmake/test_template.cmake)

target_include_directories(app PRIVATE mocks)

zephyr_library_link_libraries(suit_memptr_storage_interface)
zephyr_library_link_libraries(suit_platform_interface)
zephyr_library_link_libraries(suit_ipuc)

zephyr_library_link_libraries_ifdef(CONFIG_TEST_SUIT_PLATFORM_FETCH_VARIANT_APP suit_memory_layout_interface)
zephyr_library_link_libraries_ifdef(CONFIG_TEST_SUIT_PLATFORM_FETCH_VARIANT_SDFW suit_decrypt_test_utils)
zephyr_library_link_libraries_ifdef(CONFIG_TEST_SUIT_PLATFORM_FETCH_VARIANT_SDFW suit_stream_filters_interface)

zephyr_compile_definitions(CONFIG_SOC_NRF54H20_CPUAPP)
