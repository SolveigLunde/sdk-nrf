/*
 * Copyright (c) 2019-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef LWM2M_OS_H__
#define LWM2M_OS_H__

/**
 * @file lwm2m_os.h
 *
 * @defgroup lwm2m_carrier_os LWM2M OS layer
 * @{
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of work queues that the system must support.
 */
#define LWM2M_OS_MAX_WORK_QS 2

/**
 * @brief Maximum number of timers that the system must support.
 */
#define LWM2M_OS_MAX_TIMER_COUNT (6 + (LWM2M_OS_MAX_WORK_QS * 5))

typedef int lwm2m_os_work_q_t;
typedef int lwm2m_os_timer_t;

/**
 * @brief Maximum number of semaphores that the system must support.
 */
#define LWM2M_OS_MAX_SEM_COUNT (8 + (LWM2M_OS_MAX_WORK_QS * 1))

typedef int lwm2m_os_sem_t;

#define LWM2M_OS_LTE_MODE_NONE   -1
/* LTE Rel-13 Cat-M1 HD-FDD == E-UTRAN == LTE-M */
#define LWM2M_OS_LTE_MODE_CAT_M1  6
/* LTE Rel-13 Cat-NB1 HD-FDD || LTE Rel-14 Cat-NB1 and Cat-NB2 HD-FDD == E-UTRAN NB-S1 == NB-IoT */
#define LWM2M_OS_LTE_MODE_CAT_NB1 7

/**
 * @brief Range of the non-volatile storage identifiers used by the library.
 */
#define LWM2M_OS_STORAGE_BASE 0xCA00
#define LWM2M_OS_STORAGE_END  0xCAFF

/**
 * @brief AT command error handler callback function.
 */
typedef void (*lwm2m_os_at_handler_callback_t)(const char *notif);

/**
 * @brief Timer callback function.
 */
typedef void (*lwm2m_os_timer_handler_t)(lwm2m_os_timer_t *timer);

struct lwm2m_os_sms_deliver_address {
	char   *address_str;
	uint8_t length;
};

struct lwm2m_os_sms_udh_app_port {
	bool present;
	uint16_t dest_port;
	uint16_t src_port;
};

struct lwm2m_os_sms_deliver_header {
	struct lwm2m_os_sms_deliver_address originating_address;
	struct lwm2m_os_sms_udh_app_port    app_port;
};

union lwm2m_os_sms_header {
	struct lwm2m_os_sms_deliver_header deliver;
};

/** @brief SMS PDU data. */
struct lwm2m_os_sms_data {
	/** SMS header. */
	union lwm2m_os_sms_header header;
	/** Length of the data in data buffer. */
	int payload_len;
	/** SMS message data. */
	char *payload;
};

/**
 * @brief SMS subscriber callback function.
 */
typedef void (*lwm2m_os_sms_callback_t)(struct lwm2m_os_sms_data *const data, void *context);

/**
 * @defgroup lwm2m_os_download_evt_id LwM2M OS download events
 * @{
 */
#define LWM2M_OS_DOWNLOAD_EVT_FRAGMENT 0
#define LWM2M_OS_DOWNLOAD_EVT_ERROR    1
#define LWM2M_OS_DOWNLOAD_EVT_DONE     2
#define LWM2M_OS_DOWNLOAD_EVT_CLOSED   3
/** @} */

/**
 * @brief Download client event.
 */
struct lwm2m_os_download_evt {
	/** Event ID. */
	int id;
	union {
		/** Error cause. */
		int error;
		/** Fragment data. */
		struct lwm2m_os_fragment {
			const void *buf;
			size_t len;
		} fragment;
	};
};

/** @brief PDN family */
enum lwm2m_os_pdn_fam {
	LWM2M_OS_PDN_FAM_IPV4,
	LWM2M_OS_PDN_FAM_IPV6,
	LWM2M_OS_PDN_FAM_IPV4V6,
	LWM2M_OS_PDN_FAM_NONIP,
};

/**
 * @brief Download client configuration options.
 */
struct lwm2m_os_download_cfg {
	/** Security tag to be used for TLS. Set to NULL if non-secure. */
	const int *sec_tag_list;
	/** Number of security tags in list. Set to 0 if non-secure. */
	uint8_t sec_tag_count;
	/** PDN ID to be used for the download. */
	int pdn_id;
	/** Address family to be used for the download. */
	enum lwm2m_os_pdn_fam family;
};

/**
 * @brief Download client asynchronous event handler.
 */
typedef int (*lwm2m_os_download_callback_t)(const struct lwm2m_os_download_evt *event);

/** @brief PDN event */
enum lwm2m_os_pdn_event {
	LWM2M_OS_PDN_EVENT_CNEC_ESM,
	LWM2M_OS_PDN_EVENT_ACTIVATED,
	LWM2M_OS_PDN_EVENT_DEACTIVATED,
	LWM2M_OS_PDN_EVENT_IPV6_UP,
	LWM2M_OS_PDN_EVENT_IPV6_DOWN,
	LWM2M_OS_PDN_EVENT_NETWORK_DETACHED,
	LWM2M_OS_PDN_EVENT_APN_RATE_CONTROL_ON,
	LWM2M_OS_PDN_EVENT_APN_RATE_CONTROL_OFF,
	LWM2M_OS_PDN_EVENT_CTX_DESTROYED,
};

/**
 * @brief PDN event handler.
 *
 * If assigned during PDP context creation, the event handler will receive status information
 * relative to the Packet Data Network connection, as reported by the AT notifications CNEC and
 * GGEV.
 *
 * This handler is executed by the same context that dispatches AT notifications.
 */
typedef void (*lwm2m_os_pdn_event_handler_t)
	(uint8_t cid, enum lwm2m_os_pdn_event event, int reason);

/**
 * @brief Create a Packet Data Protocol (PDP) context.
 *
 * If a callback is provided via the @c cb parameter,
 * generate events from the CNEC and GGEV AT notifications to report
 * state of the Packet Data Network (PDN) connection.
 *
 * @param[out] cid The ID of the new PDP context.
 * @param      cb  Optional event handler.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_ctx_create(uint8_t *cid, lwm2m_os_pdn_event_handler_t cb);

/**
 * @brief Configure a Packet Data Protocol context.
 *
 * @param cid    The PDP context to configure.
 * @param apn    The Access Point Name to configure the PDP context with.
 * @param family The family to configure the PDN context for.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_ctx_configure(uint8_t cid, const char *apn, enum lwm2m_os_pdn_fam family);

/**
 * @brief Destroy a Packet Data Protocol context.
 *
 * @param cid The PDP context to destroy.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_ctx_destroy(uint8_t cid);

/**
 * @brief Activate a Packet Data Network (PDN) connection.
 *
 * @param      cid    The PDP context ID to activate a connection for.
 * @param[out] esm    If provided, the function will block to return the ESM error reason.
 * @param[out] family If provided, the function will block to return PDN_FAM_IPV4 if only IPv4 is
 *                    supported, or PDN_FAM_IPV6 if only IPv6 is supported. Otherwise, this value
 *                    will remain unchanged.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_activate(uint8_t cid, int *esm, enum lwm2m_os_pdn_fam *family);

/**
 * @brief Deactivate a Packet Data Network (PDN) connection.
 *
 * @param cid The PDP context ID.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_deactivate(uint8_t cid);

/**
 * @brief Retrieve the PDN ID for a given PDP Context.
 *
 * The PDN ID can be used to route traffic through a Packet Data Network.
 *
 * @param cid The context ID of the PDN connection.
 *
 * @return A non-negative PDN ID on success, or a negative errno otherwise.
 */
int lwm2m_os_pdn_id_get(uint8_t cid);

/**
 * @brief Set a callback for events pertaining to the default PDP context (zero).
 *
 * @param cb The PDN event handler.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_pdn_default_callback_set(lwm2m_os_pdn_event_handler_t cb);

/**
 * @brief Allocate memory.
 */
void *lwm2m_os_malloc(size_t size);

/**
 * @brief Free memory.
 */
void lwm2m_os_free(void *ptr);

/**
 * @brief Initialize a semaphore.
 *
 * @param sem           Address of the pointer to the semaphore.
 * @param initial_count Initial semaphore count.
 * @param limit         Maximum permitted semaphore count.
 *
 * @retval  0      Semaphore created successfully.
 * @retval -EINVAL Invalid values.
 */
int lwm2m_os_sem_init(lwm2m_os_sem_t **sem, unsigned int initial_count, unsigned int limit);

/**
 * @brief Take a semaphore.
 *
 * @param sem     Address of the semaphore.
 * @param timeout Timeout in milliseconds, or -1 for forever, in which case the semaphore is taken
 *                for as long as necessary.
 *
 * @retval  0      Semaphore taken.
 * @retval -EBUSY  Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
int lwm2m_os_sem_take(lwm2m_os_sem_t *sem, int timeout);

/**
 * @brief Give a semaphore.
 *
 * @param sem Address of the semaphore.
 */
void lwm2m_os_sem_give(lwm2m_os_sem_t *sem);

/**
 * @brief Reset a semaphore.
 *
 * @param sem Address of the semaphore.
 */
void lwm2m_os_sem_reset(lwm2m_os_sem_t *sem);

/**
 * @brief Get uptime, in milliseconds.
 *
 * @return Current uptime.
 */
int64_t lwm2m_os_uptime_get(void);

/**
 * @brief Get uptime delta, in milliseconds.
 *
 * @param[in] ref Pointer to a reference time, which is updated to the current
 *                uptime upon return.
 *
 * @return Elapsed time.
 */
int64_t lwm2m_os_uptime_delta(int64_t *ref);

/**
 * @brief Put a thread to sleep.
 *
 * @param ms Desired duration of sleep in milliseconds.
 *
 * @return 0 if the requested duration has elapsed, or, if the thread was woken up before the
 *         desired duration @c ms, the remaining duration left to sleep is returned.
 */
int lwm2m_os_sleep(int ms);

/**
 * @brief Reboot the system.
 */
void lwm2m_os_sys_reset(void);

/**
 * @brief Get a random value.
 *
 * @return A random 32-bit value.
 */
uint32_t lwm2m_os_rand_get(void);

/**
 * @brief Delete a non-volatile storage entry.
 *
 * @param[in] id ID of the entry to be deleted.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_storage_delete(uint16_t id);

/**
 * @brief Read an entry from non-volatile storage.
 *
 * @param[in]     id   ID of the entry to be read.
 * @param[out]    data Pointer to data buffer.
 * @param[in,out] len  Number of bytes to be read.
 *
 * @return Number of bytes read. On error, returns a negative value.
 */
int lwm2m_os_storage_read(uint16_t id, void *data, size_t len);

/**
 * @brief Write an entry to non-volatile storage.
 *
 * @param[in]     id   ID of the entry to be written.
 * @param[in]     data Pointer to the data to be written.
 * @param[in,out] len  Number of bytes to be written.
 *
 * @return Number of bytes written. On error, returns a negative value.
 */
int lwm2m_os_storage_write(uint16_t id, const void *data, size_t len);

/**
 * @brief Start a workqueue.
 *
 * @param index Number of the queue.
 * @param name  Name of the queue.
 *
 * @return Workqueue.
 */
lwm2m_os_work_q_t *lwm2m_os_work_q_start(int index, const char *name);

/**
 * @brief Reserve a timer task from the OS.
 *
 * @param handler Function to run for this task.
 * @param timer   Assigned timer task.
 */
void lwm2m_os_timer_get(lwm2m_os_timer_handler_t handler, lwm2m_os_timer_t **timer);

/**
 * @brief Release a timer task.
 *
 * @param timer Timer task to be released.
 */
void lwm2m_os_timer_release(lwm2m_os_timer_t *timer);

/**
 * @brief Start a timer on a specific queue.
 *
 * @param work_q Workqueue.
 * @param timer  Timer task.
 * @param delay  Delay before submitting the task in milliseconds.
 *
 * @retval  0      Work placed on queue, already on queue or already running.
 * @retval -EINVAL Timer or work_q not found.
 */
int lwm2m_os_timer_start_on_q(lwm2m_os_work_q_t *work_q, lwm2m_os_timer_t *timer, int64_t delay);

/**
 * @brief Cancel a timer run.
 *
 * @param timer Timer task to cancel.
 * @param sync  If true, wait for active tasks to finish before canceling.
 */
void lwm2m_os_timer_cancel(lwm2m_os_timer_t *timer, bool sync);

/**
 * @brief Obtain the time remaining on a timer.
 *
 * @param timer Timer task.
 *
 * @return Time remaining in milliseconds.
 */
int64_t lwm2m_os_timer_remaining(lwm2m_os_timer_t *timer);

/**
 * @brief Check if a timer task is pending.
 *
 * @param timer Timer task.
 *
 * @retval true  If the timer task is pending.
 * @retval false If the timer task is idle.
 */
bool lwm2m_os_timer_is_pending(lwm2m_os_timer_t *timer);

/**
 * @brief Initialize AT notification handler.
 */
void lwm2m_os_at_init(lwm2m_os_at_handler_callback_t callback);

/**
 * @brief Register as an SMS client/listener.
 *
 * @retval  0   If success.
 * @retval -EIO If unable to register as SMS listener.
 */
int lwm2m_os_sms_client_register(lwm2m_os_sms_callback_t lib_callback, void *context);

/**
 * @brief degister as an SMS client/listener.
 */
void lwm2m_os_sms_client_deregister(int handle);

/**
 * @brief Establish a connection with the server and download a file.
 *
 *  @retval 0 If the operation was successful.
 *            Otherwise, a negative error code is returned.
 */
int lwm2m_os_download_get(const char *uri, const struct lwm2m_os_download_cfg *cfg, size_t from);

/**
 * @brief Disconnect from the server.
 *
 *  @retval 0 If the operation was successful.
 *            Otherwise, a negative error code is returned.
 */
int lwm2m_os_download_disconnect(void);

/**
 * @brief Initialize the download client.
 *
 *  @retval 0 If the operation was successful.
 *            Otherwise, a negative error code is returned.
 */
int lwm2m_os_download_init(lwm2m_os_download_callback_t lib_callback);

/**
 * @brief Retrieve size of file being downloaded.
 *
 * @param[out] size Size of the file being downloaded.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int lwm2m_os_download_file_size_get(size_t *size);

/**
 * @brief Check if UICC LwM2M bootstrap is enabled.
 *
 * @retval true  If enabled
 * @retval false If disabled
 */
bool lwm2m_os_uicc_bootstrap_is_enabled(void);

/**
 * @brief Read UICC LwM2M bootstrap record.
 *
 * @param p_buffer    Buffer to store UICC LwM2M bootstrap record.
 * @param buffer_size Size of the buffer in bytes.
 *
 * @retval Length of the record. On error, returns a negative value.
 */
int lwm2m_os_uicc_bootstrap_read(uint8_t *p_buffer, int buffer_size);

/**
 * @brief get enabled system modes from modem.
 *
 * @param modes Array to store the enabled modes.
 *                  LWM2M_OS_LTE_MODE_CAT_M1  Cat-M1 (LTE-FDD)
 *                  LWM2M_OS_LTE_MODE_CAT_NB1 Cat-NB1 (NB-IoT)
 *
 * @return Number of enabled modes.
 */
size_t lwm2m_os_lte_modes_get(int32_t *modes);

/**
 * @brief set preferred bearer in modem.
 *
 * @param prefer LWM2M_OS_LTE_MODE_NONE    for no preference
 *               LWM2M_OS_LTE_MODE_CAT_M1  for Cat-M1 (LTE-FDD)
 *               LWM2M_OS_LTE_MODE_CAT_NB1 for Cat-NB1 (NB-IoT)
 */
void lwm2m_os_lte_mode_request(int32_t prefer);

/**
 * @brief Translate the error number.
 */
int lwm2m_os_nrf_errno(void);

/**
 * @defgroup lwm2m_os_dfu_img_type LwM2M OS DFU image types.
 * @{
 */
#define LWM2M_OS_DFU_IMG_TYPE_NONE             0
/**
 * MCUboot-style upgrades
 */
#define LWM2M_OS_DFU_IMG_TYPE_APPLICATION      1
/**
 * Modem delta upgrades
 */
#define LWM2M_OS_DFU_IMG_TYPE_MODEM_DELTA      2
/**
 * MCUboot-style upgrades over multiple files.
 */
#define LWM2M_OS_DFU_IMG_TYPE_APPLICATION_FILE 3
/** @} */

struct __attribute__((__packed__)) lwm2m_os_dfu_header {
	/* Number of the image file in the sequence. */
	uint8_t number;
	/* Flag indicating if the image file is the last in the sequence. */
	uint8_t is_last;
	/* Offset within the whole image in bytes. */
	uint32_t offset;
	/* Null-terminated image version. */
	char version[32];
};

#define LWM2M_OS_DFU_HEADER_MAGIC     0x424ad2dc
#define LWM2M_OS_DFU_HEADER_MAGIC_LEN sizeof(uint32_t)
#define LWM2M_OS_DFU_HEADER_LEN (LWM2M_OS_DFU_HEADER_MAGIC_LEN + sizeof(struct lwm2m_os_dfu_header))

/**
 * @brief Find the image type for the buffer of bytes received.
 *
 * @param[in]  buf    A buffer of bytes which are the start of a binary firmware image.
 * @param[in]  len    The length of the provided buffer.
 * @param[out] header DFU image header descriptor. Only applicable to MCUboot-style upgrades over
 *                    multiple files.
 *
 * @return Identifier for a supported image type or LWM2M_OS_DFU_IMG_TYPE_NONE if
 *         image type is not recognized or not supported.
 **/
int lwm2m_os_dfu_img_type(const void *const buf, size_t len, struct lwm2m_os_dfu_header **header);

/**
 * @brief Start a firmware upgrade.
 *
 * @param[in] img_type      DFU target type to be initialized.
 * @param[in] max_file_size Estimate of the new firmware image to be received. May be greater than
 *                          or equal to the actual image size received by the end.
 * @param[in] crc_validate  Flag to indicate whether to validate the incoming image fragments by
 *                          means of IEEE CRC32.
 *
 * @retval  0       Ready to start a new firmware upgrade.
 * @return  A positive number of bytes written so far, if the previous upgrade was not completed.
 *          In this case, the upgrade will resume from this offset.
 * @retval -EBUSY   Another firmware upgrade is already ongoing.
 * @retval -EFBIG   File size exceeds the DFU area size.
 * @retval -ENOTSUP Firmware image not supported or unknown.
 * @retval -EIO     Internal error.
 */
int lwm2m_os_dfu_start(int img_type, size_t max_file_size, bool crc_validate);

/**
 * @brief Receive a firmware image fragment and validate its CRC if required.
 *
 * @param[in] buf    Buffer containing the fragment.
 * @param[in] len    Length of the fragment in bytes.
 * @param[in] crc32  Expected IEEE CRC32 value to be checked for the whole fragment. Can be any
 *                   value if no validation expected.
 *
 * @retval  0       Success.
 * @retval -EACCES  lwm2m_os_dfu_start() was not called beforehand.
 * @retval -ENOMEM  Not enough memory to process the fragment.
 * @retval -EINVAL  CRC error.
 * @retval -EIO     Internal error.
 */
int lwm2m_os_dfu_fragment(const char *buf, size_t len, uint32_t crc32);

/**
 * @brief Finalize the current firmware upgrade and CRC-validate the image if required.
 *
 * @param[in] successful Indicate if upload was successful.
 * @param[in] crc32      Expected IEEE CRC32 value to be checked for the whole file in flash.
 *                       Can be any value if no validation expected.
 *
 * @retval  0       Success.
 * @retval -EACCES  lwm2m_os_dfu_start() was not called beforehand.
 * @retval -EINVAL  CRC error.
 * @retval -EIO     Internal error.
 */
int lwm2m_os_dfu_done(bool successful, uint32_t crc32);

/**
 * @brief Pause the DFU process and release the resources temporarily.
 *
 * @return  A positive number of bytes written so far.
 * @retval -EACCES  lwm2m_os_dfu_start() was not called beforehand.
 * @retval -EIO     Internal error.
 */
int lwm2m_os_dfu_pause(void);

/**
 * @brief Schedule update for uploaded image.
 *
 * @retval  0      If the update was scheduled successfully.
 * @retval -EINVAL If the data needed to perform the update was incomplete.
 * @retval -EACCES If the DFU process was not in progress.
 * @retval -EIO    Internal error.
 */
int lwm2m_os_dfu_schedule_update(void);

/**
 * @brief Reset the current DFU target.
 */
void lwm2m_os_dfu_reset(void);

/**
 * @brief Validate the application image update.
 *
 * @retval true  If the application image was updated successfully.
 * @retval false If the application image was not updated successfully.
 */
bool lwm2m_os_dfu_application_update_validate(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* LWM2M_OS_H__ */
