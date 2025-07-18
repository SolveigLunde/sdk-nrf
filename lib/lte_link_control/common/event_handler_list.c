/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>

#include <common/event_handler_list.h>

LOG_MODULE_DECLARE(lte_lc, CONFIG_LTE_LINK_CONTROL_LOG_LEVEL);

static K_MUTEX_DEFINE(list_mtx);

/**@brief List element for event handler list. */
struct event_handler {
	sys_snode_t node;
	lte_lc_evt_handler_t handler;
};

static sys_slist_t handler_list;

/**
 * @brief Find the handler from the event handler list.
 *
 * @return The node or NULL if not found and its previous node in @p prev_out.
 */
static struct event_handler *event_handler_list_node_find(struct event_handler **prev_out,
							  lte_lc_evt_handler_t handler)
{
	struct event_handler *prev = NULL, *curr;

	SYS_SLIST_FOR_EACH_CONTAINER(&handler_list, curr, node) {
		if (curr->handler == handler) {
			*prev_out = prev;
			return curr;
		}
		prev = curr;
	}
	return NULL;
}

/**@brief Add the handler in the event handler list if not already present. */
int event_handler_list_handler_append(lte_lc_evt_handler_t handler)
{
	struct event_handler *to_ins;

	k_mutex_lock(&list_mtx, K_FOREVER);

	/* Check if handler is already registered. */
	if (event_handler_list_node_find(&to_ins, handler) != NULL) {
		LOG_DBG("Handler already registered. Nothing to do");
		k_mutex_unlock(&list_mtx);
		return 0;
	}

	/* Allocate memory and fill. */
	to_ins = (struct event_handler *)k_malloc(sizeof(struct event_handler));
	if (to_ins == NULL) {
		k_mutex_unlock(&list_mtx);
		return -ENOBUFS;
	}
	memset(to_ins, 0, sizeof(struct event_handler));
	to_ins->handler = handler;

	/* Insert handler in the list. */
	sys_slist_append(&handler_list, &to_ins->node);
	k_mutex_unlock(&list_mtx);
	return 0;
}

/**@brief Remove the handler from the event handler list if registered. */
int event_handler_list_handler_remove(lte_lc_evt_handler_t handler)
{
	struct event_handler *curr, *prev = NULL;

	k_mutex_lock(&list_mtx, K_FOREVER);

	/* Check if the handler is registered before removing it. */
	curr = event_handler_list_node_find(&prev, handler);
	if (curr == NULL) {
		LOG_WRN("Handler not registered. Nothing to do");
		k_mutex_unlock(&list_mtx);
		return 0;
	}

	/* Remove the handler from the list. */
	sys_slist_remove(&handler_list, &prev->node, &curr->node);
	k_free(curr);

	k_mutex_unlock(&list_mtx);
	return 0;
}

/**@brief dispatch events. */
void event_handler_list_dispatch(const struct lte_lc_evt *const evt)
{
	struct event_handler *curr, *tmp;

	if (event_handler_list_is_empty()) {
		return;
	}

	k_mutex_lock(&list_mtx, K_FOREVER);

	/* Dispatch events to all registered handlers */
	LOG_DBG("Dispatching event: type=%d", evt->type);
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&handler_list, curr, tmp, node) {
		LOG_DBG(" - handler=0x%08X", (uint32_t)curr->handler);
		curr->handler(evt);
	}
	LOG_DBG("Done");

	k_mutex_unlock(&list_mtx);
}

/**@brief Test if the handler list is empty. */
bool event_handler_list_is_empty(void)
{
	return sys_slist_is_empty(&handler_list);
}
