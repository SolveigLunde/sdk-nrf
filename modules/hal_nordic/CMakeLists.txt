# Copyright (c) 2025 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
if(NOT DEFINED NRFX_DIR)
  set(NRFX_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/nrfx/ CACHE PATH "nrfx Directory")
endif()

zephyr_include_directories_ifdef(CONFIG_HAS_NRFX .)
add_subdirectory(${ZEPHYR_BASE}/modules/hal_nordic ${CMAKE_CURRENT_BINARY_DIR}/modules/hal_nordic)
add_subdirectory_ifdef(CONFIG_HAS_NRFX nrfx)
