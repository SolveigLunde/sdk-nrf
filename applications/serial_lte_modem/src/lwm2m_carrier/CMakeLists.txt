#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_include_directories(.)
target_sources_ifdef(CONFIG_SLM_CARRIER app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slm_at_carrier.c)
target_sources_ifdef(CONFIG_LWM2M_CARRIER_SETTINGS app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slm_at_carrier_cfg.c)
