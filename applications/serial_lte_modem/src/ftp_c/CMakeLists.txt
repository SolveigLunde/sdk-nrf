#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_include_directories(.)
target_sources_ifdef(CONFIG_SLM_FTPC app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slm_at_ftp.c)
target_sources_ifdef(CONFIG_SLM_TFTPC app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slm_at_tftp.c)
