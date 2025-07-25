/*
 * Copyright (c) 2025, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/ztest.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test);

#define WDT_WINDOW_MAX	(200)

#define DEVICE_DT_GET_AND_COMMA(node_id) DEVICE_DT_GET(node_id),
/* Generate a list of devices for all instances of the "compat" */
#define DEVS_FOR_DT_COMPAT(compat) \
	DT_FOREACH_STATUS_OKAY(compat, DEVICE_DT_GET_AND_COMMA)

static const struct device *const devices[] = {
#ifdef CONFIG_WDT_NRFX
	DEVS_FOR_DT_COMPAT(nordic_nrf_wdt)
#endif
};

typedef void (*test_func_t)(const struct device *dev);
typedef bool (*capability_func_t)(const struct device *dev);

static void setup_instance(const struct device *dev)
{
	/* Left for future test expansion. */
}

static void tear_down_instance(const struct device *dev)
{
	/* Left for future test expansion. */
}

static void test_all_instances(test_func_t func, capability_func_t capability_check)
{
	int devices_skipped = 0;

	zassert_true(ARRAY_SIZE(devices) > 0, "No device found");
	for (int i = 0; i < ARRAY_SIZE(devices); i++) {
		setup_instance(devices[i]);
		TC_PRINT("\nInstance %u: ", i + 1);
		if ((capability_check == NULL) ||
		     capability_check(devices[i])) {
			TC_PRINT("Testing %s\n", devices[i]->name);
			func(devices[i]);
		} else {
			TC_PRINT("Skipped for %s\n", devices[i]->name);
			devices_skipped++;
		}
		tear_down_instance(devices[i]);
		/* Allow logs to be printed. */
		k_sleep(K_MSEC(100));
	}
	if (devices_skipped == ARRAY_SIZE(devices)) {
		ztest_test_skip();
	}
}


/**
 * Test validates if instance can be configured.
 */
static void test_wdt_configure_instance(const struct device *dev)
{
	struct wdt_timeout_cfg default_wdt_cfg;
	int ret, wdt_chan_id;

	/* Configure Watchdog */
	default_wdt_cfg.callback = NULL;
	default_wdt_cfg.flags = WDT_FLAG_RESET_SOC;
	default_wdt_cfg.window.min = 0U;
	default_wdt_cfg.window.max = WDT_WINDOW_MAX;
	wdt_chan_id = wdt_install_timeout(dev, &default_wdt_cfg);
	zassert_true(wdt_chan_id >= 0, "%s: wdt_install_timeout() failed", dev->name);

	/* Start Watchdog */
	ret = wdt_setup(dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	zassert_equal(0, ret, "%s: wdt_setup() failed", dev->name);

	/* Check wdt_feed() */
	for (int i = 0; i < 4; i++) {
		k_busy_wait((WDT_WINDOW_MAX / 2) * 1000);
		wdt_feed(dev, wdt_chan_id);
	}

	/* Disable Watchodg */
	ret = wdt_disable(dev);
	zassert_equal(0, ret, "%s: wdt_disable() failed", dev->name);
}

static bool test_wdt_configure_capable(const struct device *dev)
{
	return true;
}

ZTEST(wdt_instances, test_wdt_configure)
{
	test_all_instances(test_wdt_configure_instance, test_wdt_configure_capable);
}

static void *suite_setup(void)
{
	int i;

	TC_PRINT("wdt_instances test executed on %s\n", CONFIG_BOARD_TARGET);
	TC_PRINT("===================================================================\n");

	for (i = 0; i < ARRAY_SIZE(devices); i++) {
		zassert_true(device_is_ready(devices[i]),
			     "Device %s is not ready", devices[i]->name);
		k_object_access_grant(devices[i], k_current_get());
	}

	return NULL;
}

ZTEST_SUITE(wdt_instances, NULL, suite_setup, NULL, NULL, NULL);
