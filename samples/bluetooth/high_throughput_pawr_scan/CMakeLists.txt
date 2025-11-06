# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(high_throughput_pawr_scan)

add_subdirectory_ifdef(CONFIG_BT_PAWR periodic_adv_rsp)
add_subdirectory_ifdef(CONFIG_BT_PAWR periodic_sync_rsp)