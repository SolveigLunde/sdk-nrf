#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
# Convert all platform and TLS/DTLS and X.509 Kconfig variables for Mbed TLS
# (strip CONFIG_)

# TF-M
kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_SPM)

# PSA core configurations
kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_CLIENT)
kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_C)
kconfig_check_and_set_base(MBEDTLS_USE_PSA_CRYPTO)
kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER)
kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_BUILTIN_KEYS)

# Platform
kconfig_check_and_set_base(MBEDTLS_PLATFORM_C)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_MEMORY)
kconfig_check_and_set_base(MBEDTLS_NO_PLATFORM_ENTROPY)
kconfig_check_and_set_base(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
kconfig_check_and_set_base(MBEDTLS_DEBUG_C)
kconfig_check_and_set_base_to_one(MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG)

# Threading configurations for CryptoCell and locally built PSA core
kconfig_check_and_set_base(MBEDTLS_THREADING_C)
kconfig_check_and_set_base(MBEDTLS_THREADING_ALT)

# Platform configurations for _ALT defines
kconfig_check_and_set_base(MBEDTLS_PLATFORM_EXIT_ALT)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_FPRINTF_ALT)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_PRINTF_ALT)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_SNPRINTF_ALT)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
kconfig_check_and_set_base(MBEDTLS_ENTROPY_HARDWARE_ALT)
kconfig_check_and_set_base(MBEDTLS_THREADING_ALT)
kconfig_check_and_set_base(MBEDTLS_PLATFORM_ZEROIZE_ALT)

# Legacy configurations for _ALT defines
kconfig_check_and_set_base(MBEDTLS_AES_SETKEY_ENC_ALT)
kconfig_check_and_set_base(MBEDTLS_AES_SETKEY_DEC_ALT)
kconfig_check_and_set_base(MBEDTLS_AES_ENCRYPT_ALT)
kconfig_check_and_set_base(MBEDTLS_AES_DECRYPT_ALT)
kconfig_check_and_set_base(MBEDTLS_AES_ALT)
kconfig_check_and_set_base(MBEDTLS_CMAC_ALT)
kconfig_check_and_set_base(MBEDTLS_CCM_ALT)
kconfig_check_and_set_base(MBEDTLS_GCM_ALT)
kconfig_check_and_set_base(MBEDTLS_CHACHA20_ALT)
kconfig_check_and_set_base(MBEDTLS_POLY1305_ALT)
kconfig_check_and_set_base(MBEDTLS_CHACHAPOLY_ALT)
kconfig_check_and_set_base(MBEDTLS_DHM_ALT)
kconfig_check_and_set_base(MBEDTLS_ECP_ALT)
kconfig_check_and_set_base(MBEDTLS_ECDH_GEN_PUBLIC_ALT)
kconfig_check_and_set_base(MBEDTLS_ECDH_COMPUTE_SHARED_ALT)
kconfig_check_and_set_base(MBEDTLS_ECDSA_GENKEY_ALT)
kconfig_check_and_set_base(MBEDTLS_ECDSA_SIGN_ALT)
kconfig_check_and_set_base(MBEDTLS_ECDSA_VERIFY_ALT)
kconfig_check_and_set_base(MBEDTLS_ECJPAKE_ALT)
kconfig_check_and_set_base(MBEDTLS_RSA_ALT)
kconfig_check_and_set_base(MBEDTLS_SHA1_ALT)
kconfig_check_and_set_base(MBEDTLS_SHA224_ALT)
kconfig_check_and_set_base(MBEDTLS_SHA256_ALT)
kconfig_check_and_set_base(MBEDTLS_SHA384_ALT)
kconfig_check_and_set_base(MBEDTLS_SHA512_ALT)

# Legacy configurations for RNG
kconfig_check_and_set_base(MBEDTLS_ENTROPY_FORCE_SHA256)
kconfig_check_and_set_base(MBEDTLS_NO_PLATFORM_ENTROPY)
kconfig_check_and_set_base_int(MBEDTLS_ENTROPY_MAX_SOURCES)

# Nordic defines for library support.
kconfig_check_and_set_base(MBEDTLS_LEGACY_CRYPTO_C)
kconfig_check_and_set_base(MBEDTLS_TLS_LIBRARY)
kconfig_check_and_set_base(MBEDTLS_X509_LIBRARY)

# Still required for some things in psa_util?
kconfig_check_and_set_base(MBEDTLS_MD_C)

# Guard against setting legacy configurations in TF-M image
if (NOT MBEDTLS_PSA_CRYPTO_SPM)
  # Platform configuration
  kconfig_check_and_set_base(MBEDTLS_ASN1_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_ASN1_WRITE_C)
  kconfig_check_and_set_base(MBEDTLS_BASE64_C)
  kconfig_check_and_set_base(MBEDTLS_OID_C)

  # PKI configurations
  kconfig_check_and_set_base(MBEDTLS_CIPHER_C)
  kconfig_check_and_set_base(MBEDTLS_PK_C)
  kconfig_check_and_set_base(MBEDTLS_PKCS5_C)
  kconfig_check_and_set_base(MBEDTLS_PK_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_PK_WRITE_C)
  kconfig_check_and_set_base(MBEDTLS_PEM_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_PEM_WRITE_C)

  # TLS/DTLS configurations
  kconfig_check_and_set_base(MBEDTLS_SSL_ALL_ALERT_MESSAGES)
  kconfig_check_and_set_base(MBEDTLS_SSL_ALL_ALERT_MESSAGES)
  kconfig_check_and_set_base(MBEDTLS_SSL_DTLS_CONNECTION_ID)
  kconfig_check_and_set_base(MBEDTLS_SSL_CONTEXT_SERIALIZATION)
  kconfig_check_and_set_base(MBEDTLS_SSL_DEBUG_ALL)
  kconfig_check_and_set_base(MBEDTLS_SSL_ENCRYPT_THEN_MAC)
  kconfig_check_and_set_base(MBEDTLS_SSL_EXTENDED_MASTER_SECRET)
  kconfig_check_and_set_base(MBEDTLS_SSL_KEEP_PEER_CERTIFICATE)
  kconfig_check_and_set_base(MBEDTLS_SSL_RENEGOTIATION)
  kconfig_check_and_set_base(MBEDTLS_SSL_MAX_FRAGMENT_LENGTH)
  kconfig_check_and_set_base(MBEDTLS_SSL_PROTO_TLS1_2)
  kconfig_check_and_set_base(MBEDTLS_SSL_PROTO_TLS1_3)
  kconfig_check_and_set_base(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_SSL_TLS1_3_COMPATIBILITY_MODE)
  kconfig_check_and_set_base(MBEDTLS_SSL_PROTO_DTLS)
  kconfig_check_and_set_base(MBEDTLS_SSL_ALPN)
  kconfig_check_and_set_base(MBEDTLS_SSL_DTLS_ANTI_REPLAY)
  kconfig_check_and_set_base(MBEDTLS_SSL_DTLS_HELLO_VERIFY)
  kconfig_check_and_set_base(MBEDTLS_SSL_DTLS_SRTP)
  kconfig_check_and_set_base(MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE)
  kconfig_check_and_set_base(MBEDTLS_SSL_SESSION_TICKETS)
  kconfig_check_and_set_base(MBEDTLS_SSL_EXPORT_KEYS)
  kconfig_check_and_set_base(MBEDTLS_SSL_SERVER_NAME_INDICATION)
  kconfig_check_and_set_base(MBEDTLS_SSL_VARIABLE_BUFFER_LENGTH)
  kconfig_check_and_set_base(MBEDTLS_SSL_CACHE_C)
  kconfig_check_and_set_base(MBEDTLS_SSL_TICKET_C)
  kconfig_check_and_set_base(MBEDTLS_SSL_CLI_C)
  kconfig_check_and_set_base(MBEDTLS_SSL_SRV_C)
  kconfig_check_and_set_base(MBEDTLS_SSL_TLS_C)
  kconfig_check_and_set_base(MBEDTLS_SSL_COOKIE_C)

  kconfig_check_and_set_base_int(MBEDTLS_SSL_IN_CONTENT_LEN)
  kconfig_check_and_set_base_int(MBEDTLS_SSL_OUT_CONTENT_LEN)
  kconfig_check_and_set_base(MBEDTLS_SSL_CIPHERSUITES)
  kconfig_check_and_set_base(MBEDTLS_SSL_EXTENDED_MASTER_SECRET)

  kconfig_check_and_set_base_int(MBEDTLS_MPI_WINDOW_SIZE)
  kconfig_check_and_set_base_int(MBEDTLS_MPI_MAX_SIZE)

  # x509 configurations
  kconfig_check_and_set_base(MBEDTLS_X509_RSASSA_PSS_SUPPORT)
  kconfig_check_and_set_base(MBEDTLS_X509_USE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CRT_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CRL_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CSR_PARSE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CREATE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CRT_WRITE_C)
  kconfig_check_and_set_base(MBEDTLS_X509_CSR_WRITE_C)

  # KRKNWK-20181
  kconfig_check_and_set_base(MBEDTLS_SSL_CLI_ALLOW_WEAK_CERTIFICATE_VERIFICATION_WITHOUT_HOSTNAME)

  # TLS key exchange
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_PSK_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_RSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED)
  kconfig_check_and_set_base(MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED)
endif()

kconfig_check_and_set_base(MBEDTLS_PSA_CRYPTO_CONFIG)

# Generate the Mbed TLS config file (default nrf-config.h)
configure_file(${NRF_SECURITY_ROOT}/configs/nrf-config.h.template
  ${generated_include_path}/${CONFIG_MBEDTLS_CFG_FILE}
)
