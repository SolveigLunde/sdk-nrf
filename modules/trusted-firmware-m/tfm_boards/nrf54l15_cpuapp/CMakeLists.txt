#
# Copyright (c) 2021, Nordic Semiconductor ASA.
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

set(NRF_BOARD_SELECTED True)

add_subdirectory(${Trusted\ Firmware\ M_SOURCE_DIR}/platform/ext/target/nordic_nrf/common/nrf54l15 nrf54l15)

add_subdirectory(.. tfm_board)

target_include_directories(platform_s
  PUBLIC
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/nrf_security/src/drivers/cracen/cracenpsa/include
  )

target_sources(platform_s
  PRIVATE
  ${ZEPHYR_BASE}/soc/nordic/nrf54l/soc.c
  )

target_include_directories(platform_s
  PRIVATE
  ${ZEPHYR_BASE}/modules/cmsis/
  ${ZEPHYR_BASE}/soc/nordic/nrf54l
  ${ZEPHYR_BASE}/soc/nordic/common
  )

# nrf54l15_application.h should be defining __ICACHE_PRESENT, but
# it is not, until this is fixed we define it here.
target_compile_definitions(platform_s
  PRIVATE
  __ICACHE_PRESENT=1
  )

install(FILES       ${CMAKE_CURRENT_LIST_DIR}/ns/cpuarch_ns.cmake
        DESTINATION ${INSTALL_PLATFORM_NS_DIR}
        RENAME      cpuarch.cmake)

install(FILES       config.cmake
        DESTINATION ${INSTALL_PLATFORM_NS_DIR})

install(FILES       ../common/config.cmake
        DESTINATION ${INSTALL_PLATFORM_NS_DIR}/../common/)

install(DIRECTORY   ${Trusted\ Firmware\ M_SOURCE_DIR}/platform/ext/target/nordic_nrf/nrf54l15dk_nrf54l15_cpuapp/tests
        DESTINATION ${INSTALL_PLATFORM_NS_DIR}
)
