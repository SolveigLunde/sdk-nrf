#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(shell_utils.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_ONOFF_CLI shell_onoff_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LVL_CLI shell_lvl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_DTT_CLI shell_dtt_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_PONOFF_CLI shell_ponoff_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_PLVL_CLI shell_plvl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_BATTERY_CLI shell_bat_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LOC_CLI shell_loc_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_PROP_CLI shell_prop_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LIGHTNESS_CLI shell_lightness_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LIGHT_CTL_CLI shell_ctl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LIGHT_HSL_CLI shell_hsl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LIGHT_XYL_CLI shell_xyl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_LIGHT_CTRL_CLI shell_ctrl_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_TIME_CLI shell_time_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_SCENE_CLI shell_scene_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_SCHEDULER_CLI shell_scheduler_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_SENSOR_CLI shell_sensor_cli.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SHELL_DM_CLI shell_dm_cli.c)
