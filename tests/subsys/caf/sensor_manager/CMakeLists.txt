#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Sensor Manager sample")
# Include application event headers
zephyr_library_include_directories(src/events)
zephyr_library_include_directories(src/modules)
# Add include directory for board specific CAF def files
zephyr_include_directories(configuration/common)

# Add test sources
target_sources(app PRIVATE src/main.c)

add_subdirectory(src/events)
add_subdirectory(src/modules)
