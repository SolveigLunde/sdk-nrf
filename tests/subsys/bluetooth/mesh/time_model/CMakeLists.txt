#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bt_mesh_time_model_test)

target_include_directories(app PUBLIC
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh
  ${ZEPHYR_BASE}/subsys/bluetooth
  ../common
  )

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE
  ${app_sources}
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh/time_srv.c
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh/time.c
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh/time_util.c
  )

target_compile_options(app
  PRIVATE
  -DCONFIG_BT_MESH_MODEL_KEY_COUNT=5
  -DCONFIG_BT_MESH_MODEL_GROUP_COUNT=5
  -DCONFIG_BT_LOG_LEVEL=0
  -DCONFIG_BT_MESH_TIME_SRV=1
  -DCONFIG_BT_MESH_TIME_SRV_CLOCK_ACCURACY=500
  -DCONFIG_BT_MESH_TIME_MESH_HOP_UNCERTAINTY=30
  -DCONFIG_BT_MESH_USES_MBEDTLS_PSA=1
  )

zephyr_ld_options(
    ${LINKERFLAGPREFIX},--allow-multiple-definition
    )
