# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grtc_clk_out)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
