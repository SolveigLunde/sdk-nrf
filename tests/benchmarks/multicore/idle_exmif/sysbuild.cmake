#
# Copyright (c) 2025 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if("${SB_CONFIG_REMOTE_BOARD}" STREQUAL "")
  message(FATAL_ERROR "REMOTE_BOARD must be set to a valid board name")
endif()

# Add remote project
ExternalZephyrProject_Add(
    APPLICATION remote
    SOURCE_DIR ${SYSBUILD_NRF_MODULE_DIR}/tests/benchmarks/power_consumption/common/remote_sleep_forever
    BOARD ${SB_CONFIG_REMOTE_BOARD}
    BOARD_REVISION ${BOARD_REVISION}
  )

# Add a dependency so that the remote image will be built and flashed first
add_dependencies(idle_exmif remote)
# Add dependency so that the remote image is flashed first.
sysbuild_add_dependencies(FLASH idle_exmif remote)
