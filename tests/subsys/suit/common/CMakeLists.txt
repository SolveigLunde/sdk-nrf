#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

add_subdirectory(mci_test)
add_subdirectory(validator_test)
add_subdirectory_ifdef(CONFIG_SUIT_STREAM_FILTER_DECRYPT decrypt_utils)
zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

if(CONFIG_MBEDTLS)
  if(CONFIG_SUIT_PLATFORM)
    # Link SUIT platform (incl. crypto) module with mbedTLS library, that provides PSA crypto APIs.
    target_link_libraries(suit_platform_interface INTERFACE mbedTLS)
  endif()

  if(CONFIG_SUIT_STREAM_SINK_DIGEST)
    target_link_libraries(suit_stream_sinks_interface INTERFACE mbedTLS)
  endif()

  if(CONFIG_SUIT_STREAM_FILTER_DECRYPT)
    target_link_libraries(suit_stream_filters_interface INTERFACE mbedTLS)
    target_link_libraries(suit_decrypt_test_utils PUBLIC mbedTLS)
  endif()

  if(CONFIG_SUIT_STORAGE_LAYOUT_SOC)
    target_link_libraries(suit_storage_interface INTERFACE mbedTLS)
  endif()

  # Fix the mbedTLS build scripts:
  #   The "mbedtls/library/psa_crypto_cipher.c" file includes "<psa_crypto_cipher.h>",
  #   but this header is not a part of mbedTLS public API, thus it is not found and
  #   the compiler reports this as an error.
  target_include_directories(mbedTLS INTERFACE ${ZEPHYR_MBEDTLS_MODULE_DIR}/library)
endif()

if(CONFIG_SUIT_DEFAULT_MBEDTLS_CONFIG)
  # Add include path for the default, customized mbedTLS configuration header.
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/tls_config)
endif()

zephyr_interface_library_named(suit_update_magic_values)
target_include_directories(suit_update_magic_values INTERFACE .)
