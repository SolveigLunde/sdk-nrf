#
# Copyright (c) 2025 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_rtio_loopback)

target_sources(app PRIVATE ${ZEPHYR_BASE}/samples/drivers/i2c/rtio_loopback/src/main.c)
