#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_sources_ifdef(CONFIG_BT_GATT_POOL gatt_pool.c)
zephyr_sources_ifdef(CONFIG_BT_GATT_DM gatt_dm.c)
zephyr_sources_ifdef(CONFIG_BT_SCAN scan.c)
zephyr_sources_ifdef(CONFIG_BT_CONN_CTX conn_ctx.c)
zephyr_sources_ifdef(CONFIG_BT_ENOCEAN enocean.c)
zephyr_sources_ifdef(CONFIG_BT_LL_SOFTDEVICE_HEADERS_INCLUDE hci_vs_sdc.c)

add_subdirectory_ifdef(CONFIG_BT_ADV_PROV adv_prov)
add_subdirectory_ifdef(CONFIG_BT_LL_SOFTDEVICE controller)
add_subdirectory_ifdef(CONFIG_BT_CHANNEL_SOUNDING cs_de)
add_subdirectory_ifdef(CONFIG_BT_MESH mesh)

add_subdirectory_ifdef(CONFIG_BT_NRF_SERVICES services)

add_subdirectory_ifdef(CONFIG_BT_RPC rpc)

add_subdirectory(host_extensions)
