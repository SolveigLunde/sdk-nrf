#ifndef PAWR_ONBOARDING_H_
#define PAWR_ONBOARDING_H_

#include <stdbool.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define PAWR_MAX_RESPONSE_SLOTS CONFIG_BT_MAX_THROUGHPUT_DEVICES
#else
#define PAWR_MAX_RESPONSE_SLOTS 1
#endif
#define PAWR_DEFAULT_SUBEVENT   0

/* UUIDs for the onboarding GATT service on the synchronizer side */
#define BT_UUID_PAWR_ONBOARDING_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_PAWR_SLOT_CHAR_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_PAWR_ONBOARDING_SERVICE BT_UUID_DECLARE_128(BT_UUID_PAWR_ONBOARDING_SERVICE_VAL)
#define BT_UUID_PAWR_SLOT_CHAR        BT_UUID_DECLARE_128(BT_UUID_PAWR_SLOT_CHAR_VAL)

struct pawr_slot_config {
	uint8_t subevent;
	uint8_t response_slot;
	uint16_t payload_id;
	uint8_t total_slots;
} __packed;

#endif /* PAWR_ONBOARDING_H_ */

