#ifndef PAWR_BROADCASTER_H_
#define PAWR_BROADCASTER_H_

#include <zephyr/bluetooth/bluetooth.h>
#include <stdint.h>
#include <stdbool.h>

int pawr_broadcaster_init(struct bt_le_ext_adv **out_adv);

void pawr_broadcaster_set_slot_expected(uint8_t slot, bool expected);

uint8_t pawr_broadcaster_total_slots(void);

#endif /* PAWR_BROADCASTER_H_ */

