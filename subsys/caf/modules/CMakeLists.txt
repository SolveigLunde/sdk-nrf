#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_CAF_BLE_ADV ble_adv.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BLE_BOND ble_bond.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BLE_STATE ble_state.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BLE_STATE_PM ble_state_pm.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BUTTONS buttons.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BUTTONS_PM_KEEP_ALIVE buttons_pm_keep_alive.c)

zephyr_library_sources_ifdef(CONFIG_CAF_CLICK_DETECTOR click_detector.c)

zephyr_library_sources_ifdef(CONFIG_CAF_FACTORY_RESET_REQUEST factory_reset_request.c)

zephyr_library_sources_ifdef(CONFIG_CAF_LEDS leds.c)

zephyr_library_sources_ifdef(CONFIG_CAF_NET_STATE_LTE net_state_lte.c)

zephyr_library_sources_ifdef(CONFIG_CAF_NET_STATE_OT net_state_ot.c)

zephyr_library_sources_ifdef(CONFIG_CAF_POWER_MANAGER power_manager.c)

zephyr_library_sources_ifdef(CONFIG_CAF_SENSOR_MANAGER sensor_manager.c)

zephyr_library_sources_ifdef(CONFIG_CAF_SENSOR_DATA_AGGREGATOR sensor_data_aggregator.c)

zephyr_library_sources_ifdef(CONFIG_CAF_SETTINGS_LOADER settings_loader.c)

zephyr_library_sources_ifdef(CONFIG_CAF_BLE_SMP ble_smp.c)

zephyr_library_sources_ifdef(CONFIG_CAF_SHELL caf_shell.c)

zephyr_library_link_libraries_ifdef(CONFIG_MCUMGR mgmt_mcumgr)
