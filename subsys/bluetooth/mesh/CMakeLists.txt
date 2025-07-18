#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(model_utils.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_ONOFF_SRV gen_onoff_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_ONOFF_CLI gen_onoff_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LVL_SRV gen_lvl_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LVL_CLI gen_lvl_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_DTT_SRV gen_dtt_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_DTT_CLI gen_dtt_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_PONOFF_SRV gen_ponoff_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_PONOFF_CLI gen_ponoff_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_PLVL_SRV gen_plvl_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_PLVL_CLI gen_plvl_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_BATTERY_SRV gen_battery_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_BATTERY_CLI gen_battery_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LOC_SRV gen_loc_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LOC_CLI gen_loc_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_PROP_SRV gen_prop_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_PROP_CLI gen_prop_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHTNESS_SRV lightness_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHTNESS_CLI lightness_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTRL_SRV light_ctrl_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTRL_REG light_ctrl_reg.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTRL_REG_SPEC light_ctrl_reg_spec.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTRL_CLI light_ctrl_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_DK_PROV dk_prov.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_SRV sensor_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_CLI sensor_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor_types.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_TIME_SRV time_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_TIME_CLI time_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_TIME time.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_TIME time_util.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTL_CLI light_ctl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_TEMP_SRV light_temp_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_CTL_SRV light_ctl_srv.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_HSL light_hsl.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_HSL_CLI light_hsl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_HSL_SRV light_hsl_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_SAT_SRV light_sat_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_HUE_SRV light_hue_srv.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SCENE_SRV scene_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SCENE_CLI scene_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_XYL_CLI light_xyl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_LIGHT_XYL_SRV light_xyl_srv.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SCHEDULER_CLI scheduler_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SCHEDULER_SRV scheduler_srv.c)

add_subdirectory_ifdef(CONFIG_BT_MESH_VENDOR_MODELS vnd)
add_subdirectory_ifdef(CONFIG_BT_MESH_SHELL shell)

zephyr_linker_sources(SECTIONS sensor_types.ld)
zephyr_linker_sources(SECTIONS scene_types.ld)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_RPL_STORAGE_MODE_EMDS rpl.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_KEY_IMPORTER key_importer.c)

if(CONFIG_BT_MESH_USES_MBEDTLS_PSA AND (NOT CONFIG_SOC_FAMILY_NORDIC_NRF))
  zephyr_library_link_libraries(mbedTLS)
endif()
