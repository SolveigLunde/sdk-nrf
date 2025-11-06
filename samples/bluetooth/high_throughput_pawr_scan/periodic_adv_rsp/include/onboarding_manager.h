#ifndef ONBOARDING_MANAGER_H_
#define ONBOARDING_MANAGER_H_

#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>

int onboarding_manager_init(struct bt_le_ext_adv *pawr_adv);

void onboarding_manager_start(void);

void onboarding_manager_handle_slot_idle(uint8_t slot);

#endif /* ONBOARDING_MANAGER_H_ */

