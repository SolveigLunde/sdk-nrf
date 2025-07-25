#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# This file creates variables with the magic numbers used for firmware metadata
# as comma-separated lists of numbers.

math(EXPR
  MAGIC_COMPATIBILITY_VALIDATION_INFO
  "(${CONFIG_SB_VALIDATION_INFO_VERSION}) |
   (${CONFIG_FW_INFO_HARDWARE_ID} << 8) |
   (${CONFIG_SB_VALIDATION_INFO_CRYPTO_ID} << 16) |
   (${CONFIG_FW_INFO_MAGIC_COMPATIBILITY_ID} << 24)"
  )

set(VALIDATION_INFO_MAGIC    "${CONFIG_FW_INFO_MAGIC_COMMON},${CONFIG_SB_VALIDATION_INFO_MAGIC},${MAGIC_COMPATIBILITY_VALIDATION_INFO}")
set(VALIDATION_POINTER_MAGIC "${CONFIG_FW_INFO_MAGIC_COMMON},${CONFIG_SB_VALIDATION_POINTER_MAGIC},${MAGIC_COMPATIBILITY_VALIDATION_INFO}")

if(TARGET zephyr_interface)
  zephyr_compile_definitions(
    VALIDATION_INFO_MAGIC=${VALIDATION_INFO_MAGIC}
    VALIDATION_POINTER_MAGIC=${VALIDATION_POINTER_MAGIC}
    )
endif()
