#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bt_mesh_silvair_enocean_test)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app
    PRIVATE
    ${ZEPHYR_BASE}/subsys/bluetooth/common/addr.c
    ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh/vnd/silvair_enocean_srv.c
    )

target_include_directories(app
    PRIVATE
    ${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh
    ${ZEPHYR_BASE}/subsys/bluetooth
    )

target_compile_options(app
    PRIVATE
    -DCONFIG_BT_ENOCEAN_STORE=1
    -DCONFIG_BT_MESH_SILVAIR_ENOCEAN_AUTO_COMMISSION=1
    -DCONFIG_BT_MESH_MODEL_KEY_COUNT=1
    -DCONFIG_BT_MESH_MODEL_GROUP_COUNT=1
    -DCONFIG_BT_MESH_USES_MBEDTLS_PSA=1
    )

zephyr_ld_options(
    ${LINKERFLAGPREFIX},--allow-multiple-definition
    )
