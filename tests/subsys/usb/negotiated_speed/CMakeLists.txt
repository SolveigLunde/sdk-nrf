# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mass)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
FILE(GLOB app_sources ${ZEPHYR_BASE}/samples/subsys/usb/mass/src/main.c)
target_sources(app PRIVATE ${app_sources})

if(CONFIG_BUILD_WITH_TFM)
  target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/api_ns/interface/include
  )
endif()
