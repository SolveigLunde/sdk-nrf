#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_wifi_radio_test)

set(NRF_WIFI_DRV_BASE ${WEST_TOPDIR}/nrf/drivers/wifi/nrf700x)

target_include_directories(app PRIVATE
	inc
)

target_sources(app PRIVATE
	src/nrf_wifi_radio_test_main.c
	src/nrf_wifi_radio_test_shell.c
	src/nrf_wifi_radio_ficr_shell.c
	src/ficr_prog.c
)

target_link_libraries(app PRIVATE nrf-wifi-osal)
