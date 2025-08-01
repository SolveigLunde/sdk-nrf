/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "test_events.h"
#include "data_event.h"
#include "order_event.h"

#include "test_config.h"

#define MODULE test_basic

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_test_start_event(aeh)) {
		struct test_start_event *st = cast_test_start_event(aeh);

		switch (st->test_id) {
		case TEST_BASIC:
		{
			struct test_end_event *et = new_test_end_event();

			et->test_id = st->test_id;
			APP_EVENT_SUBMIT(et);
			break;
		}

		case TEST_DATA:
		{
			struct data_event *event = new_data_event();
			static char descr[] = TEST_STRING;

			event->val1 = TEST_VAL1;
			event->val2 = TEST_VAL2;
			event->val3 = TEST_VAL3;
			event->val1u = TEST_VAL1U;
			event->val2u = TEST_VAL2U;
			event->val3u = TEST_VAL3U;

			event->descr = descr;

			APP_EVENT_SUBMIT(event);
			break;
		}

		case TEST_EVENT_ORDER:
		{
			for (size_t i = 0; i < TEST_EVENT_ORDER_CNT; i++) {
				struct order_event *event = new_order_event();

				event->val = i;
				APP_EVENT_SUBMIT(event);
			}
			break;
		}

		case TEST_SUBSCRIBER_ORDER:
		{
			struct order_event *event = new_order_event();

			APP_EVENT_SUBMIT(event);
			break;
		}

		default:
			/* Ignore other test cases, check if proper test_id. */
			zassert_true(st->test_id < TEST_CNT,
				     "test_id out of range");
			break;
		}

		return false;
	}

	zassert_true(false, "Event unhandled");

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, test_start_event);
