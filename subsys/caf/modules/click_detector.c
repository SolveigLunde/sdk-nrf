/*
 * Copyright (c) 2019-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>

#define MODULE click_detector
#include <caf/events/module_state_event.h>

#include <caf/events/button_event.h>
#include <caf/events/power_event.h>
#include <caf/events/click_event.h>

#include CONFIG_CAF_CLICK_DETECTOR_DEF_PATH

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_CAF_CLICK_DETECTOR_LOG_LEVEL);

#define CLICK_CHECK_PERIOD	100	/* ms */

#define SHORT_CLICK_MAX CONFIG_CAF_CLICK_DETECTOR_SHORT_CLICK_MSEC	/* ms */
#define LONG_CLICK_MIN CONFIG_CAF_CLICK_DETECTOR_LONG_CLICK_MSEC	/* ms */

#define TIMER_INACTIVE		-1


enum state {
	STATE_DISABLED,
	STATE_ACTIVE,
	STATE_OFF,
};

struct key_state {
	int32_t click_short_timeout;
	int32_t pressed_time;
};

static struct key_state keys[ARRAY_SIZE(click_detector_config)];

static enum state state;
static struct k_work_delayable click_check;


static void submit_click_event(uint16_t key_id, enum click click)
{
	struct click_event *event = new_click_event();

	event->key_id = key_id;
	event->click = click;

	APP_EVENT_SUBMIT(event);
}

static enum click get_click(int32_t time_diff)
{
	if (time_diff < SHORT_CLICK_MAX) {
		return CLICK_SHORT;
	}

	if (time_diff > LONG_CLICK_MIN) {
		return CLICK_LONG;
	}

	return CLICK_NONE;
}

static bool update_click_short_timeout(struct key_state *key, uint16_t key_id)
{
	if (key->click_short_timeout != TIMER_INACTIVE) {
		key->click_short_timeout -= CLICK_CHECK_PERIOD;

		if (key->click_short_timeout <= 0) {
			submit_click_event(key_id, CLICK_SHORT);
			key->click_short_timeout = TIMER_INACTIVE;
		}

		return true;
	}

	return false;
}

static bool update_click_time(struct key_state *key, uint16_t key_id)
{
	if (key->pressed_time != TIMER_INACTIVE) {
		key->pressed_time += CLICK_CHECK_PERIOD;

		if (get_click(key->pressed_time) == CLICK_LONG) {
			key->pressed_time = TIMER_INACTIVE;
			submit_click_event(key_id, CLICK_LONG);
		}

		return true;
	}

	return false;
}

static void click_check_fn(struct k_work *work)
{
	__ASSERT_NO_MSG(state == STATE_ACTIVE);

	bool any_processed = false;

	for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
		uint16_t key_id = click_detector_config[i].key_id;

		if (update_click_short_timeout(&keys[i], key_id)) {
		       any_processed = true;
		}

		if (update_click_time(&keys[i], key_id)) {
			any_processed = true;
		}
	}

	if (any_processed) {
		k_work_reschedule(&click_check, K_MSEC(CLICK_CHECK_PERIOD));
	}
}

static void init(void)
{
	BUILD_ASSERT(ARRAY_SIZE(keys) == ARRAY_SIZE(click_detector_config));

	for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
		keys[i].pressed_time = TIMER_INACTIVE;
		keys[i].click_short_timeout = TIMER_INACTIVE;
	}

	k_work_init_delayable(&click_check, click_check_fn);

	state = STATE_ACTIVE;
}

static void process_key(struct key_state *key, uint16_t key_id, bool pressed)
{
	__ASSERT_NO_MSG(state == STATE_ACTIVE);

	if (!pressed && (key->pressed_time != TIMER_INACTIVE)) {
		enum click click = get_click(key->pressed_time);

		if (click != CLICK_SHORT) {
			submit_click_event(key_id, click);
		} else if (key->click_short_timeout != TIMER_INACTIVE) {
			submit_click_event(key_id, CLICK_DOUBLE);
			key->click_short_timeout = TIMER_INACTIVE;
		} else {
			key->click_short_timeout = SHORT_CLICK_MAX;
			k_work_reschedule(&click_check, K_MSEC(CLICK_CHECK_PERIOD));
		}

		key->pressed_time = TIMER_INACTIVE;
	} else if (pressed) {
		key->pressed_time = 0;
		k_work_reschedule(&click_check, K_MSEC(CLICK_CHECK_PERIOD));
	}
}

static bool button_event_handler(const struct button_event *event)
{
	bool ret = false;

	for (size_t i = 0; i < ARRAY_SIZE(click_detector_config); i++) {
		if (event->key_id == click_detector_config[i].key_id) {
			process_key(&keys[i], event->key_id, event->pressed);
			ret = click_detector_config[i].consume_button_event;
		}
	}

	return ret;
}

static void power_down(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
		keys[i].click_short_timeout = TIMER_INACTIVE;
		keys[i].pressed_time = TIMER_INACTIVE;
	}

	state = STATE_OFF;
	module_set_state(MODULE_STATE_OFF);
	k_work_cancel_delayable(&click_check);
}

static void wake_up(void)
{
	state = STATE_ACTIVE;
	module_set_state(MODULE_STATE_READY);
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_button_event(aeh)) {
		return button_event_handler(cast_button_event(aeh));
	}

	if (is_module_state_event(aeh)) {
		const struct module_state_event *event =
			cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			__ASSERT_NO_MSG(state == STATE_DISABLED);

			init();
			module_set_state(MODULE_STATE_READY);
		}

		return false;
	}

	if (IS_ENABLED(CONFIG_CAF_CLICK_DETECTOR_PM_EVENTS) &&
	    is_power_down_event(aeh)) {
		if (state != STATE_OFF) {
			power_down();
		}
		return false;
	}

	if (IS_ENABLED(CONFIG_CAF_CLICK_DETECTOR_PM_EVENTS) &&
	    is_wake_up_event(aeh)) {
		if (state != STATE_ACTIVE) {
			wake_up();
		}
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}
APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, button_event);
#if CONFIG_CAF_CLICK_DETECTOR_PM_EVENTS
APP_EVENT_SUBSCRIBE(MODULE, power_down_event);
APP_EVENT_SUBSCRIBE(MODULE, wake_up_event);
#endif /* CONFIG_CAF_CLICK_DETECTOR_PM_EVENTS */
