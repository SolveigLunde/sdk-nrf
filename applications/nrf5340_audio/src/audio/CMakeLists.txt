#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/audio_system.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/audio_datapath.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/sw_codec_select.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/le_audio_rx.c
)
