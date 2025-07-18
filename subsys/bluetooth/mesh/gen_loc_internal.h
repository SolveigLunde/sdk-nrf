/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @defgroup bt_mesh_loc_internal Generic Location Model internals
 * @{
 * @brief Internal defines and functions for the Generic Location models.
 */
#ifndef BT_MESH_GEN_LOC_INTERNAL_H__
#define BT_MESH_GEN_LOC_INTERNAL_H__

#include <math.h>
#include <bluetooth/mesh/gen_loc.h>

#define BT_MESH_LOC_LATITUDE_NOT_CONFIGURED  0x80000000
#define BT_MESH_LOC_LONGITUDE_NOT_CONFIGURED 0x80000000

static inline void bt_mesh_loc_global_encode(struct net_buf_simple *buf,
					     const struct bt_mesh_loc_global *loc)
{
	int32_t latitude = BT_MESH_LOC_LATITUDE_NOT_CONFIGURED;
	int32_t longitude = BT_MESH_LOC_LONGITUDE_NOT_CONFIGURED;

	if (loc->latitude >= -90.0 && loc->latitude <= 90.0) {
		latitude = floor(loc->latitude / 90 * (INT32_MAX - 1));
	}

	if (loc->longitude >= -180.0 && loc->longitude <= 180.0) {
		longitude = floor(loc->longitude / 180 * (INT32_MAX - 1));
	}

	net_buf_simple_add_le32(buf, latitude);
	net_buf_simple_add_le32(buf, longitude);
	net_buf_simple_add_le16(buf, loc->altitude);
}

static inline void bt_mesh_loc_global_decode(struct net_buf_simple *buf,
					     struct bt_mesh_loc_global *loc)
{
	int32_t latitude = net_buf_simple_pull_le32(buf);
	int32_t longitude = net_buf_simple_pull_le32(buf);

	loc->latitude = latitude == BT_MESH_LOC_LATITUDE_NOT_CONFIGURED
				? BT_MESH_LOC_GLOBAL_LATITUDE_UNKNOWN
				: ((double)latitude * 90) / (INT32_MAX - 1);
	loc->longitude = longitude == BT_MESH_LOC_LONGITUDE_NOT_CONFIGURED
				 ? BT_MESH_LOC_GLOBAL_LONGITUDE_UNKNOWN
				 : ((double)longitude * 180) / (INT32_MAX - 1);
	loc->altitude = net_buf_simple_pull_le16(buf);
}

static inline uint8_t bt_mesh_loc_4bit_log2_encode(uint32_t value)
{
	for (uint8_t x = 0; x < 15; x++) {
		if (value <= (125U << x)) {
			return x;
		}
	}
	return BIT_MASK(4U);
}

static inline void bt_mesh_loc_local_encode(struct net_buf_simple *buf,
					    const struct bt_mesh_loc_local *loc)
{
	net_buf_simple_add_le16(buf, loc->north);
	net_buf_simple_add_le16(buf, loc->east);
	net_buf_simple_add_le16(buf, loc->altitude);

	uint8_t enc_floor;

	switch (loc->floor_number) {
	case BT_MESH_LOC_FLOOR_NUMBER_GROUND_FLOOR_0:
	case BT_MESH_LOC_FLOOR_NUMBER_GROUND_FLOOR_1:
	case BT_MESH_LOC_FLOOR_NUMBER_UNKNOWN:
		enc_floor = loc->floor_number;
		break;
	default:
		if (loc->floor_number > BT_MESH_LOC_FLOOR_NUMBER_MAX) {
			enc_floor = BT_MESH_LOC_FLOOR_NUMBER_MAX;
		} else if (loc->floor_number < BT_MESH_LOC_FLOOR_NUMBER_MIN) {
			enc_floor = BT_MESH_LOC_FLOOR_NUMBER_MIN;
		} else {
			enc_floor = loc->floor_number + 20;
		}
		break;
	}
	net_buf_simple_add_u8(buf, enc_floor);

	/* Encoding for the update time and precision is t = 2^(x - 3) */
	uint8_t enc_update_time =
		loc->time_delta >= 0 ?
			bt_mesh_loc_4bit_log2_encode(loc->time_delta) :
			BIT_MASK(4U);
	uint8_t enc_precision = bt_mesh_loc_4bit_log2_encode(loc->precision_mm);

	uint16_t uncertainty = (loc->is_mobile) |
			    ((enc_update_time & BIT_MASK(4U)) << 8U) |
			    ((enc_precision & BIT_MASK(4U)) << 12U);

	net_buf_simple_add_le16(buf, uncertainty);
}

static inline void bt_mesh_loc_local_decode(struct net_buf_simple *buf,
					    struct bt_mesh_loc_local *loc)
{
	loc->north = net_buf_simple_pull_le16(buf);
	loc->east = net_buf_simple_pull_le16(buf);
	loc->altitude = net_buf_simple_pull_le16(buf);

	uint8_t enc_floor = net_buf_simple_pull_u8(buf);

	switch (enc_floor) {
	case BT_MESH_LOC_FLOOR_NUMBER_GROUND_FLOOR_0:
	case BT_MESH_LOC_FLOOR_NUMBER_GROUND_FLOOR_1:
	case BT_MESH_LOC_FLOOR_NUMBER_UNKNOWN:
		loc->floor_number = enc_floor;
		break;
	default:
		loc->floor_number = enc_floor - 20;
		break;
	}

	uint16_t uncertainty = net_buf_simple_pull_le16(buf);

	loc->is_mobile = uncertainty & BIT_MASK(1);
	loc->time_delta = (125 << ((uncertainty >> 8) & BIT_MASK(4)));
	loc->precision_mm = (125 << ((uncertainty >> 12) & BIT_MASK(4)));
}

#endif /* BT_MESH_GEN_LOC_INTERNAL_H__ */

/** @} */
