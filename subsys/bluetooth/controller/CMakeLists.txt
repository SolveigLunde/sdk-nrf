#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(
  hci_driver.c
  hci_internal.c
  hci_internal_wrappers.c
)

zephyr_library_sources_ifdef(
  CONFIG_BT_CTLR_CRYPTO
  crypto.c
)

zephyr_library_sources_ifdef(
  CONFIG_BT_CTLR_SDC_CS_MULTIPLE_ANTENNA_SUPPORT
  cs_antenna_switch.c
)

zephyr_library_link_libraries(subsys__bluetooth)

zephyr_include_directories(.)
