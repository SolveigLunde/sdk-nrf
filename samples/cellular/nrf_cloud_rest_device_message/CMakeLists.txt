#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nRF_Cloud_REST_Device_Message_Sample)

zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})
zephyr_include_directories(include/)

# NORDIC SDK APP START
# Add include directory for board specific CAF def files
zephyr_include_directories(
  include/${NORMALIZED_BOARD_TARGET}
  )
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END
