#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

add_subdirectory(app_tools)
add_subdirectory_ifdef(CONFIG_SUIT plat_err)
add_subdirectory_ifdef(CONFIG_SUIT memory_layout)
add_subdirectory_ifdef(CONFIG_SUIT_ORCHESTRATOR orchestrator)
add_subdirectory_ifdef(CONFIG_SUIT_PLATFORM platform)
add_subdirectory_ifdef(CONFIG_SUIT_MEMPTR_STORAGE memptr_storage)
add_subdirectory_ifdef(CONFIG_SUIT_STORAGE storage)
add_subdirectory_ifdef(CONFIG_SUIT_STREAM stream)
add_subdirectory_ifdef(CONFIG_SUIT_MCI mci)
add_subdirectory_ifdef(CONFIG_SUIT_METADATA metadata)
add_subdirectory_ifdef(CONFIG_SUIT_UTILS utils)
add_subdirectory_ifdef(CONFIG_SUIT_CACHE cache)
add_subdirectory_ifdef(CONFIG_SUIT_DFU orchestrator_app)
add_subdirectory_ifdef(CONFIG_SUIT_ENVELOPE_INFO envelope_info)
add_subdirectory_ifdef(CONFIG_SUIT_EXECUTION_MODE execution_mode)
add_subdirectory_ifdef(CONFIG_SUIT_VALIDATOR validator)
add_subdirectory_ifdef(CONFIG_SUIT_EVENTS events)
add_subdirectory_ifdef(CONFIG_SUIT_MANIFEST_VARIABLES manifest_variables)
add_subdirectory_ifdef(CONFIG_SUIT_IPUC ipuc)
