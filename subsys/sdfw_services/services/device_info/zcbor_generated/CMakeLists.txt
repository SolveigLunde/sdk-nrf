#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

#
# Generated using cmake macro generate_and_add_cbor_files.
#

zephyr_sources(
	device_info_service_decode.c
	device_info_service_encode.c
)

zephyr_include_directories(.)
