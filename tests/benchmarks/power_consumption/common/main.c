/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);

static bool state = true;
extern void thread_definition(void);

/* Some tests require that test thread controls the moment when it is
 * suspended. In that case test implements this function and returns true to
 * indicated that test thread will take case of the suspension and it can
 * be skipped in the common code.
 */
__weak bool self_suspend_req(void)
{
	return false;
}

K_THREAD_DEFINE(thread_id, 500, thread_definition, NULL, NULL, NULL,
				5, 0, 0);

void timer_handler(struct k_timer *dummy)
{
	if (state == true) {
		state = false;
		gpio_pin_set_dt(&led, 1);
		k_thread_resume(thread_id);
	} else {
		state = true;
		gpio_pin_set_dt(&led, 0);
		if (self_suspend_req() == false) {
			k_thread_suspend(thread_id);
		}
	}
}

K_TIMER_DEFINE(timer, timer_handler, NULL);

int main(void)
{
	int rc;

	rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	__ASSERT_NO_MSG(rc == 0);

	k_timer_start(&timer, K_SECONDS(1), K_SECONDS(1));

	while (1) {
		k_msleep(1000);
	}
	return 0;
}
