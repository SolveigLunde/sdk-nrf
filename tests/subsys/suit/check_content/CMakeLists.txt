#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(integration_test_suit_plat_check_content)
include(../cmake/test_template.cmake)

zephyr_library_link_libraries(suit_platform_interface)
zephyr_library_link_libraries(suit_memptr_storage_interface)
zephyr_library_link_libraries(suit_stream_sinks_interface)
zephyr_library_link_libraries(suit_manifest_variables)
