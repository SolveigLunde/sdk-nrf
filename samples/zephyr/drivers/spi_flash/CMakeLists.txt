# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi_flash)

target_sources(app PRIVATE ${ZEPHYR_BASE}/samples/drivers/spi_flash/src/main.c)
