#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# SUIT Manifest Configuration Information API
zephyr_interface_library_named(suit_mci)
target_include_directories(suit_mci INTERFACE include)
target_link_libraries(suit_mci INTERFACE suit_platform_err)
target_link_libraries(suit_mci INTERFACE suit_metadata)

zephyr_library()

if(CONFIG_SUIT_MCI_IMPL_SDFW)
  zephyr_library_sources(src/suit_mci_${CONFIG_SOC}.c)
endif()

zephyr_library_sources(src/suit_generic_ids.c)

zephyr_library_link_libraries(suit_mci)
zephyr_library_link_libraries(suit_utils)
zephyr_library_link_libraries_ifdef(CONFIG_SUIT_STORAGE suit_storage_interface)
zephyr_library_link_libraries(suit_execution_mode)
zephyr_library_link_libraries(suit)
