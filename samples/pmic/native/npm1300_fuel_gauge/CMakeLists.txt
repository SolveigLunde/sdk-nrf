# Copyright (c) 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(SHIELD npm1300_ek)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(npm1300_fuel_gauge)

add_subdirectory(src)
