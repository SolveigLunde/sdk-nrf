#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_CBOR src/nrf_provisioning_cbor_decode.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_CBOR src/nrf_provisioning_cbor_encode.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_CODEC src/nrf_provisioning_codec.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_HTTP src/nrf_provisioning_http.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_SHELL src/nrf_provisioning_shell.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_WITH_CERT src/cert_amazon_root_ca1.c src/cert_coap_root_ca.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_COAP src/nrf_provisioning_coap.c)
zephyr_library_sources_ifdef(CONFIG_NRF_PROVISIONING_CUSTOM_AT src/nrf_provisioning_custom_at.c)
zephyr_library_sources(src/nrf_provisioning.c src/nrf_provisioning_at.c src/nrf_provisioning_jwt.c)
zephyr_include_directories(./include)
