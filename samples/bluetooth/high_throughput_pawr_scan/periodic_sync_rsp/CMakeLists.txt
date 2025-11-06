# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(periodic_sync_rsp)

# Enable floating point printf
target_compile_definitions(app PRIVATE CONFIG_CBPRINTF_FP_SUPPORT=y)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE
  include
  ../periodic_adv_rsp/include
  )