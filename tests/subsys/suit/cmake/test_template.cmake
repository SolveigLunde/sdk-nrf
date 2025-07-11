#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE
  ${app_sources}
  )

if (VERBOSE)
  zephyr_compile_definitions(ZCBOR_VERBOSE)
endif()

if (ASSERTS)
  zephyr_compile_definitions(ZCBOR_ASSERTS)
endif()

zephyr_compile_options(-Werror)

if (CONFIG_64BIT)
  set(bit_arg -b 64)
endif()

set(SUIT_SUBSYS_DIR ${ZEPHYR_NRF_MODULE_DIR}/subsys/suit)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../common" "${PROJECT_BINARY_DIR}/test_common")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../mocks" "${PROJECT_BINARY_DIR}/test_mocks")
