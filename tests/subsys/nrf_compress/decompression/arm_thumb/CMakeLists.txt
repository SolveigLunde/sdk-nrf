#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(decompression_arm_thumb)

target_sources(app PRIVATE src/main.c)

generate_inc_file_for_target(
  app
  ${ZEPHYR_NRFXLIB_MODULE_DIR}/tests/subsys/nrf_compress/decompression/arm_thumb.dat
  ${ZEPHYR_BINARY_DIR}/include/generated/arm_thumb.inc
  )

generate_inc_file_for_target(
  app
  ${ZEPHYR_NRFXLIB_MODULE_DIR}/tests/subsys/nrf_compress/decompression/arm_thumb_compressed.dat
  ${ZEPHYR_BINARY_DIR}/include/generated/arm_thumb_compressed.inc
  )
