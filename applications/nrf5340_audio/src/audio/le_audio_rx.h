/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _LE_AUDIO_RX_H_
#define _LE_AUDIO_RX_H_

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/net_buf.h>
#include <audio_defines.h>

/**
 * @brief	Data handler when ISO data has been received.
 *
 * @param[in]	audio_frame_rx	Pointer to the audio buffer.
 * @param[in]	meta		Pointer to the audio metadata.
 * @param[in]	channel_index	Which channel is received.
 */
void le_audio_rx_data_handler(struct net_buf *audio_frame_rx, struct audio_metadata *meta,
			      uint8_t channel_index);

/**
 * @brief	Initialize the receive audio path.
 *
 * @return	0 if successful, error otherwise.
 */
int le_audio_rx_init(void);

#endif /* _LE_AUDIO_RX_H_ */
