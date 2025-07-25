/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <modem/modem_slm.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(mdm_slm_mon, CONFIG_MODEM_SLM_LOG_LEVEL);

struct at_notif_fifo {
	void *fifo_reserved;
	char data[]; /* Null-terminated AT notification string */
};

static void slm_monitor_task(struct k_work *work);

static K_FIFO_DEFINE(at_monitor_fifo);
static K_HEAP_DEFINE(at_monitor_heap, 1024);
static K_WORK_DEFINE(at_monitor_work, slm_monitor_task);

extern char *strnstr(const char *haystack, const char *needle, size_t haystack_sz);

static bool is_paused(const struct slm_monitor_entry *mon)
{
	return mon->paused;
}

static bool has_match(const struct slm_monitor_entry *mon, const char *notif)
{
	return (mon->filter == MON_ANY || strstr(notif, mon->filter));
}

/* Schedules a workqueue to dispatch AT notifications. */
void slm_monitor_dispatch(const char *notif, size_t len)
{
	struct at_notif_fifo *at_notif;
	size_t sz_needed;
	size_t notif_len;
	char *match = NULL;

	/* TODO:
	 * To reliably separate AT notifications from AT commands, data and other
	 * AT notifications would require that the AT notifications are separated with control
	 * characters or possibly sent with a separate CMUX channel.
	 */
	STRUCT_SECTION_FOREACH(slm_monitor_entry, e) {
		if (!is_paused(e)) {
			match = strnstr(notif, e->filter, len);
			if (match) {
				notif_len = len - (size_t)(match - notif);
				break;
			}
		}
	}

	if (!match) {
		/* Only copy monitored notifications to save heap */
		return;
	}

	sz_needed = sizeof(struct at_notif_fifo) + notif_len + sizeof(char);

	at_notif = k_heap_alloc(&at_monitor_heap, sz_needed, K_NO_WAIT);
	if (!at_notif) {
		LOG_WRN("No heap space for incoming notification: %s",
			notif);
		return;
	}

	strncpy(at_notif->data, match, notif_len);
	at_notif->data[notif_len] = '\0';

	k_fifo_put(&at_monitor_fifo, at_notif);
	k_work_submit(&at_monitor_work);
}

static void slm_monitor_task(struct k_work *work)
{
	struct at_notif_fifo *at_notif;

	while ((at_notif = k_fifo_get(&at_monitor_fifo, K_NO_WAIT))) {
		/* Match notification with all monitors */
		LOG_DBG("AT notif: %.*s", strlen(at_notif->data) - strlen("\r\n"), at_notif->data);
		STRUCT_SECTION_FOREACH(slm_monitor_entry, e) {
			if (!is_paused(e) && has_match(e, at_notif->data)) {
				LOG_DBG("Dispatching to %p", e->handler);
				e->handler(at_notif->data);
			}
		}
		k_heap_free(&at_monitor_heap, at_notif);
	}
}
