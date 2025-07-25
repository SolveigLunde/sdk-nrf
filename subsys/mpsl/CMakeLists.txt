#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if(CONFIG_MPSL AND NOT CONFIG_MPSL_FEM_ONLY)
  add_subdirectory(init)
endif()

if(CONFIG_MPSL_FEM OR CONFIG_MPSL_FEM_PIN_FORWARDER OR CONFIG_MPSL_FEM_API_AVAILABLE)
  add_subdirectory(fem)
endif()

add_subdirectory(cx)

if(CONFIG_MPSL OR CONFIG_MPSL_FEM_ONLY)
  add_subdirectory(hwres)
endif()

if(CONFIG_MPSL_USE_ZEPHYR_PM)
  add_subdirectory(pm)
endif()

if(CONFIG_MPSL_USE_EXTERNAL_CLOCK_CONTROL)
  add_subdirectory(clock_ctrl)
endif()

add_subdirectory_ifdef(CONFIG_MPSL_PIN_DEBUG pin_debug)
