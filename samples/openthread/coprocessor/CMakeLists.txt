#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

set(OT_NCP_VENDOR_HOOK_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(openthread_coprocessor)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c src/nrf_802154_radio_wrapper.c)
target_sources_ifdef(CONFIG_RCP_SAMPLE_HCI app PRIVATE src/rcp_hci.c)
# NORDIC SDK APP END
