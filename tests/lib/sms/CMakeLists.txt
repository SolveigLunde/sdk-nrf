#
# Copyright (c) 2021 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sms_test)

# generate runner for the test
test_runner_generate(src/sms_test.c)

cmock_handle(${ZEPHYR_NRFXLIB_MODULE_DIR}/nrf_modem/include/nrf_modem_at.h
	     FUNC_EXCLUDE ".*nrf_modem_at_scanf"
	     FUNC_EXCLUDE ".*nrf_modem_at_printf"
	     WORD_EXCLUDE "__nrf_modem_(printf|scanf)_like\(.*\)")

# When mocking nrf_modem_at then nrf_modem/include must manually be added
# because CONFIG_NRF_MODEM_LINK_BINARY=n
zephyr_include_directories(${ZEPHYR_NRFXLIB_MODULE_DIR}/nrf_modem/include/)

# add test file
target_sources(app PRIVATE src/sms_test.c)
