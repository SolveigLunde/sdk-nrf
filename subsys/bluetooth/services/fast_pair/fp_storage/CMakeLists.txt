#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library_named(fp_storage)

if(CONFIG_BT_FAST_PAIR_STORAGE_MANAGER)
  zephyr_linker_sources(SECTIONS fp_storage_manager.ld)
  target_sources(fp_storage PRIVATE fp_storage_manager.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_AK_BACKEND_STANDARD)
  target_sources(fp_storage PRIVATE fp_storage_ak.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_AK_BACKEND_MINIMAL)
  target_sources(fp_storage PRIVATE fp_storage_ak_minimal.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_PN)
  target_sources(fp_storage PRIVATE fp_storage_pn.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_USER_RESET_ACTION)
  target_sources(fp_storage PRIVATE fp_storage_user_reset_action.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_FMDN_CLOCK)
  target_sources(fp_storage PRIVATE fp_storage_clock.c)
endif()
if(CONFIG_BT_FAST_PAIR_STORAGE_FMDN_EIK)
  target_sources(fp_storage PRIVATE fp_storage_eik.c)
endif()

target_include_directories(fp_storage PUBLIC include)
target_include_directories(fp_storage PUBLIC ../include/common)
if(CONFIG_BT_FAST_PAIR_STORAGE_EXPOSE_PRIV_API)
  target_include_directories(fp_storage PUBLIC include_priv)
else()
  target_include_directories(fp_storage PRIVATE include_priv)
endif()

# For strnlen()
zephyr_library_compile_definitions(_POSIX_C_SOURCE=200809L)
