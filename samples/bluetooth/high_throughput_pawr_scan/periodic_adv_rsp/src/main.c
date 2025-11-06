#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>

#include "pawr_broadcaster.h"
#include "onboarding_manager.h"

int main(void)
{
	int err;
	struct bt_le_ext_adv *pawr_adv = NULL;

	printk("Starting PAwR advertiser with deterministic onboarding\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	err = pawr_broadcaster_init(&pawr_adv);
	if (err) {
		printk("PAwR broadcaster init failed (err %d)\n", err);
		return err;
	}

	err = onboarding_manager_init(pawr_adv);
    if (err) {
		printk("Onboarding manager init failed (err %d)\n", err);
		return err;
	}

	onboarding_manager_start();

    while (true) {
        k_sleep(K_SECONDS(1));
    }

	return 0;
}

