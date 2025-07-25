/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef LOCATION_H_
#define LOCATION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#if (defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_AGNSS)) ||\
	defined(CONFIG_LOCATION_DATA_DETAILS)
#include <nrf_modem_gnss.h>
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_PGPS)
#include <net/nrf_cloud_pgps.h>
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_LOCATION_METHOD_CELLULAR)
#include <modem/lte_lc.h>
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_LOCATION_METHOD_WIFI)
#include <net/wifi_location_common.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file location.h
 * @brief Public APIs for the Location library.
 * @defgroup location Location library
 * @{
 */

/** Location method. */
enum location_method {
	/** LTE cellular positioning. */
	LOCATION_METHOD_CELLULAR = 1,
	/** Global Navigation Satellite System (GNSS). */
	LOCATION_METHOD_GNSS,
	/** Wi-Fi positioning. */
	LOCATION_METHOD_WIFI,
	/**
	 * Wi-Fi and cellular positioning combined into one cloud request.
	 *
	 * Cannot be used in the method list passed to location_request(),
	 * but this method can appear in the location event (@ref location_event_data.id)
	 * to indicate that Wi-Fi and cellular positioning methods have been combined
	 * into a single cloud location method.
	 *
	 * If the combined method is desired, set Wi-Fi and cellular methods next to each other
	 * in the method list passed to location_request(). This also happens by default if
	 * NULL configuration is passed to location_request().
	 */
	LOCATION_METHOD_WIFI_CELLULAR,
};

/** Location acquisition mode. */
enum location_req_mode {
	/** Fallback to next preferred method (default). */
	LOCATION_REQ_MODE_FALLBACK = 0,
	/** All requested methods are used sequentially. */
	LOCATION_REQ_MODE_ALL,
};

/** Event IDs. */
enum location_event_id {
	/** Location update. */
	LOCATION_EVT_LOCATION = 1,
	/** Getting location timed out. */
	LOCATION_EVT_TIMEOUT,
	/** An error occurred when trying to get the location. */
	LOCATION_EVT_ERROR,
	/**
	 * Application has indicated that getting location has been completed,
	 * the result is not known, and the Location library does not need to care about it.
	 *
	 * This event can occur only if @kconfig{CONFIG_LOCATION_SERVICE_EXTERNAL} is set.
	 */
	LOCATION_EVT_RESULT_UNKNOWN,
	/**
	 * GNSS is requesting A-GNSS data.
	 *
	 * Application should obtain the data and send it to location_agnss_data_process().
	 */
	LOCATION_EVT_GNSS_ASSISTANCE_REQUEST,
	/**
	 * GNSS is requesting P-GPS data.
	 *
	 * Application should obtain the data and send it to location_pgps_data_process().
	 */
	LOCATION_EVT_GNSS_PREDICTION_REQUEST,
	/**
	 * Cloud location request with neighbor cell and/or Wi-Fi access point information
	 * is available.
	 *
	 * The application should send the information to cloud services and then call
	 * location_cloud_location_ext_result_set().
	 */
	LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST,
	/**
	 * Location request has been started.
	 *
	 * This event is only sent if @kconfig{CONFIG_LOCATION_DATA_DETAILS} is set.
	 */
	LOCATION_EVT_STARTED,
	/**
	 * A fallback from one method to another has occurred,
	 * and the positioning procedure continues.
	 *
	 * This event is only sent if @kconfig{CONFIG_LOCATION_DATA_DETAILS} is set and
	 * @ref location_config.mode is @ref LOCATION_REQ_MODE_FALLBACK.
	 */
	LOCATION_EVT_FALLBACK,
};

/** Result of the external cloud location request. */
enum location_ext_result {
	/** Cloud location request was successful. */
	LOCATION_EXT_RESULT_SUCCESS,
	/**
	 * The result of the cloud location request is unknown.
	 *
	 * From the fallback functionality perspective, this is handled similarly to
	 * @ref LOCATION_EXT_RESULT_ERROR.
	 */
	LOCATION_EXT_RESULT_UNKNOWN,
	/** Cloud location request failed. */
	LOCATION_EXT_RESULT_ERROR
};

/** Location accuracy. */
enum location_accuracy {
	/** Allow lower accuracy to conserve power. */
	LOCATION_ACCURACY_LOW,
	/** Normal accuracy. */
	LOCATION_ACCURACY_NORMAL,
	/** Try to improve accuracy. Increases power consumption. */
	LOCATION_ACCURACY_HIGH,
};

/** Date and time (UTC). */
struct location_datetime {
	/** True if date and time are valid, false if not. */
	bool valid;
	/** 4-digit representation (Gregorian calendar). */
	uint16_t year;
	/** 1...12 */
	uint8_t month;
	/** 1...31 */
	uint8_t day;
	/** 0...23 */
	uint8_t hour;
	/** 0...59 */
	uint8_t minute;
	/** 0...59 */
	uint8_t second;
	/** 0...999 */
	uint16_t ms;
};

#if defined(CONFIG_LOCATION_DATA_DETAILS)
/** Location details for GNSS. */
struct location_data_details_gnss {
	/** Number of satellites tracked at the time of event. */
	uint8_t satellites_tracked;
	/** Number of satellites used at the time of event. */
	uint8_t satellites_used;
	/** PVT data. */
	struct nrf_modem_gnss_pvt_data_frame pvt_data;
	/**
	 * Elapsed GNSS time in milliseconds.
	 *
	 * This is the time since last start of GNSS operation until the fix or timeout
	 * including any time LTE has blocked GNSS.
	 *
	 * @ref pvt_data member has ``execution_time``, which indicates cumulative GNSS
	 * execution time since last start excluding any time LTE has blocked GNSS.
	 */
	uint32_t elapsed_time_gnss;
};

/** Location details for cellular. */
struct location_data_details_cellular {
	/** Number of neighbor cells. */
	uint8_t ncells_count;
	/** Number of GCI (surrounding) cells. */
	uint8_t gci_cells_count;
};

/** Location details for Wi-Fi. */
struct location_data_details_wifi {
	/** Number of Wi-Fi APs. */
	uint16_t ap_count;
};

/**
 * Location details.
 *
 * Only one of the child structures is filled most of the time depending on the method
 * found in @ref location_event_data.method member of the parent structure.
 * For a combined method @ref LOCATION_METHOD_WIFI_CELLULAR both @ref cellular and
 * @ref wifi are filled.
 */
struct location_data_details {
	/**
	 * Elapsed method time in milliseconds.
	 *
	 * This is the time from method start until it completes and includes any time
	 * spent waiting for some conditions to happen before proceeding, such as
	 * waiting for LTE connection to go idle for some methods.
	 */
	uint32_t elapsed_time_method;

#if defined(CONFIG_LOCATION_METHOD_GNSS)
	/** Location details for GNSS. */
	struct location_data_details_gnss gnss;
#endif
#if defined(CONFIG_LOCATION_METHOD_CELLULAR)
	/** Location details for cellular. */
	struct location_data_details_cellular cellular;
#endif
#if defined(CONFIG_LOCATION_METHOD_WIFI)
	/** Location details for Wi-Fi. */
	struct location_data_details_wifi wifi;
#endif
};
#endif

/** Location data. */
struct location_data {
	/** Geodetic latitude (deg) in WGS-84. */
	double latitude;
	/** Geodetic longitude (deg) in WGS-84. */
	double longitude;
	/** Location accuracy in meters (1-sigma). */
	float accuracy;
	/** Date and time (UTC). */
	struct location_datetime datetime;

#if defined(CONFIG_LOCATION_DATA_DETAILS)
	/** Location details. */
	struct location_data_details details;
#endif
};

#if defined(CONFIG_LOCATION_DATA_DETAILS)
/** Location error information. */
struct location_data_error {
	/** Data details at the time of error. */
	struct location_data_details details;
};

/** Information for an unknown location result. */
struct location_data_unknown {
	/** Data details at the time of an unknown location result. */
	struct location_data_details details;
};

/** Location fallback information. */
struct location_data_fallback {
	/** New location method that is tried next. */
	enum location_method next_method;
	/**
	 * Event ID indicating the cause for the fallback.
	 *
	 * Either @ref LOCATION_EVT_TIMEOUT and @ref LOCATION_EVT_ERROR.
	 */
	enum location_event_id cause;
	/** Data details at the time of a timeout or an error that caused the fallback. */
	struct location_data_details details;
};
#endif

/** Cloud location information. */
struct location_data_cloud {
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_LOCATION_METHOD_CELLULAR)
	/** Cellular cell information. */
	const struct lte_lc_cells_info *cell_data;
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_LOCATION_METHOD_WIFI)
	/** Wi-Fi access point information. */
	const struct wifi_scan_info *wifi_data;
#endif
};

/** Location event data. */
struct location_event_data {
	/** Event ID. */
	enum location_event_id id;
	/** Used location method. */
	enum location_method method;

	/** Event specific data. */
	union {
		/** Current location, used with event @ref LOCATION_EVT_LOCATION. */
		struct location_data location;

#if defined(CONFIG_LOCATION_DATA_DETAILS)
		/**
		 * Relevant location data when a timeout or an error occurs.
		 * Used with events @ref LOCATION_EVT_TIMEOUT and @ref LOCATION_EVT_ERROR.
		 */
		struct location_data_error error;

		/**
		 * Relevant location data when an unknown result occurs.
		 * Used with event @ref LOCATION_EVT_RESULT_UNKNOWN.
		 */
		struct location_data_unknown unknown;

		/**
		 * Relevant location data when a fallback to another method occurs
		 * due to a timeout or an error.
		 * Used with event @ref LOCATION_EVT_FALLBACK.
		 */
		struct location_data_fallback fallback;
#endif

#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_AGNSS)
		/**
		 * A-GNSS notification data frame used by GNSS to let the application know it needs
		 * new assistance data, used with event @ref LOCATION_EVT_GNSS_ASSISTANCE_REQUEST.
		 */
		struct nrf_modem_gnss_agnss_data_frame agnss_request;
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_PGPS)
		/**
		 * P-GPS notification data frame used by GNSS to let the application know it needs
		 * new assistance data, used with event @ref LOCATION_EVT_GNSS_PREDICTION_REQUEST.
		 */
		struct gps_pgps_request pgps_request;
#endif
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) &&\
	(defined(CONFIG_LOCATION_METHOD_CELLULAR) || defined(CONFIG_LOCATION_METHOD_WIFI))
		/**
		 * Cloud location information to let the application know it should send these
		 * to a cloud service to resolve the location.
		 * Used with event @ref LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST.
		 */
		struct location_data_cloud cloud_location_request;
#endif
	};
};

/** GNSS configuration. */
struct location_gnss_config {
	/**
	 * @brief Timeout (in milliseconds), meaning how long GNSS is allowed to run when trying to
	 * acquire a fix. SYS_FOREVER_MS means that the timer is disabled, meaning that GNSS will
	 * continue to search until it gets a fix or the application calls cancel.
	 *
	 * @details Note that this is not real time as experienced by the user.
	 * Since GNSS cannot run while LTE is operating, the running time is counted from
	 * the instant when GNSS is allowed to start. Library waits until RRC connection
	 * is inactive before starting GNSS. If LTE power saving mode (PSM) is enabled
	 * and A-GNSS is disabled, library waits until modem enters PSM before starting GNSS,
	 * thus maximizing uninterrupted operating window and minimizing power consumption.
	 *
	 * Default value is 120000 (2 minutes). It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_GNSS_TIMEOUT} configuration.
	 */
	int32_t timeout;

	/** @brief Desired accuracy level.
	 *
	 * @details If accuracy is set to @ref LOCATION_ACCURACY_LOW, GNSS relaxes the
	 * criteria for a qualifying fix and may produce fixes using only three satellites.
	 *
	 * If accuracy is set to @ref LOCATION_ACCURACY_HIGH, instead of using the first fix,
	 * GNSS is allowed to run for a longer time. This typically improves the location accuracy.
	 *
	 * Default value is @ref LOCATION_ACCURACY_NORMAL. It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_GNSS_ACCURACY} choice configuration.
	 */
	enum location_accuracy accuracy;

	/**
	 * @brief The number of fixes GNSS is allowed to produce before the library outputs the
	 * current location when accuracy is set to LOCATION_ACCURACY_HIGH.
	 *
	 * @details If accuracy is set to LOCATION_ACCURACY_NORMAL or LOCATION_ACCURACY_LOW this
	 * parameter has no effect.
	 *
	 * Default value is 3. It is applied when location_config_defaults_set()
	 * function is called and can be changed at build time with
	 * @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_GNSS_NUM_CONSECUTIVE_FIXES} configuration.
	 */
	uint8_t num_consecutive_fixes;

	/**
	 * @brief Obstructed visibility detection. If set to true, the library tries to detect
	 * situations where getting a GNSS fix is unlikely or would consume a lot of energy.
	 * When such a situation is detected, GNSS is stopped without waiting for a fix or a
	 * timeout.
	 *
	 * @details See Kconfig for related configuration options.
	 *
	 * Default value is false. It is applied when location_config_defaults_set()
	 * function is called and can be changed at build time with
	 * @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_GNSS_VISIBILITY_DETECTION} configuration.
	 *
	 * @note Only supported with modem firmware v1.3.2 or later.
	 */
	bool visibility_detection;

	/**
	 * @brief Enable GNSS priority mode if GNSS does not get enough runtime due to LTE idle mode
	 * operations.
	 *
	 * @details If set to true, the library triggers GNSS priority mode if five consecutive PVT
	 * messages indicate that GNSS is blocked by LTE idle mode operations. This is especially
	 * helpful if A-GNSS or P-GPS is not enabled or downloading assistance data fails and GNSS
	 * module has to decode navigation data from the satellite broadcast. Priority mode is
	 * disabled automatically after the first fix or after 40 seconds.
	 *
	 * If the device attempts to send data during the priority mode, it will be buffered and
	 * sent after the priority time window ends. In case of mobile terminated data reception
	 * during the priority mode the network will typically buffer the data and sent them to the
	 * device once the priority time window ends. However, it is possible that the network drops
	 * the data, or some protocol timer expires causing data transfer to fail.
	 *
	 * Default value is false. It is applied when location_config_defaults_set()
	 * function is called and can be changed at build time with
	 * @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_GNSS_PRIORITY_MODE} configuration.
	 */
	bool priority_mode;
};

/** LTE cellular positioning configuration. */
struct location_cellular_config {
	/**
	 * @brief Timeout (in milliseconds) of how long the cellular positioning procedure can take.
	 * SYS_FOREVER_MS means that the timer is disabled.
	 *
	 * @details Default value is 30000 (30 seconds). It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_CELLULAR_TIMEOUT} configuration.
	 *
	 * Timeout only applies to neighbor cell search, not the cloud communication.
	 * Timeout for the entire location request specified in @ref location_config structure
	 * is still valid.
	 * When @kconfig{CONFIG_LOCATION_SERVICE_EXTERNAL} is enabled, this timeout stops when
	 * event @ref LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST is sent. However, timeout specified in
	 * @ref location_config structure is still valid.
	 */
	int32_t timeout;

	/**
	 * @brief Number of cells to be requested for cellular positioning.
	 *
	 * @details Default value is 4. It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_CELLULAR_CELL_COUNT} configuration.
	 *
	 * Maximum value is 15.
	 *
	 * The number of cells mean GCI cells including current cell.
	 * Normal neighbor cells are received and used but they do not count towards the cell count.
	 * If the number of cells is bigger than one, GCI (surrounding) cells are requested.
	 * Hence, zero and one mean that only normal neighbor cell search is performed
	 * but no GCI search.
	 *
	 * Note that even if there are a lot of cells available, the number of cells
	 * used for positioning may be lower than the requested number of cells due to
	 * the behavior of the search algorithm. Also, the number of cells used for
	 * positioning may be higher than the requested number of cells if there are
	 * more cells in the history information (including current cell).
	 */
	uint8_t cell_count;
};

/** Wi-Fi positioning configuration. */
struct location_wifi_config {
	/**
	 * @brief Timeout (in milliseconds) of how long the Wi-Fi positioning procedure can take.
	 * SYS_FOREVER_MS means that the timer is disabled.
	 *
	 * @details Default value is 30000 (30 seconds). It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_WIFI_TIMEOUT} configuration.
	 *
	 * Timeout only applies to Wi-Fi scan, not the cloud communication.
	 * Timeout for the entire location request specified in @ref location_config structure
	 * is still valid.
	 *
	 * When @kconfig{CONFIG_LOCATION_SERVICE_EXTERNAL} is enabled, this timeout stops when
	 * event @ref LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST is sent. However, timeout specified in
	 * @ref location_config structure is still valid.
	 */
	int32_t timeout;
};

/** Location method configuration. */
struct location_method_config {
	/** Location method. */
	enum location_method method;
	union {
		/** Configuration for @ref LOCATION_METHOD_CELLULAR. */
		struct location_cellular_config cellular;
		/** Configuration for @ref LOCATION_METHOD_GNSS. */
		struct location_gnss_config gnss;
		/** Configuration for @ref LOCATION_METHOD_WIFI. */
		struct location_wifi_config wifi;
	};
};

/** Location request configuration. */
struct location_config {
	/** Number of location methods in 'methods'. */
	uint8_t methods_count;

	/**
	 * @brief Selected location methods and associated configurations in priority order.
	 *
	 * @details Index 0 has the highest priority. Number of used methods is indicated in
	 * 'methods_count' and it can be smaller than the size of this table.
	 *
	 * Wi-Fi and cellular scan results are combined into single cloud request, that is,
	 * these methods are handled together, if the following conditions are met:
	 *   - Methods are one after the other in location request method list
	 *   - @ref mode is @ref LOCATION_REQ_MODE_FALLBACK
	 */
	struct location_method_config methods[CONFIG_LOCATION_METHODS_LIST_SIZE];

	/**
	 * @brief Position update interval in seconds.
	 *
	 * @details Set to 0 for a single position update. For periodic position updates
	 * the valid range is 1...65535 seconds.
	 *
	 * Default value is 0. It is applied when
	 * location_config_defaults_set() function is called and can be changed at build time
	 * with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_INTERVAL} configuration.
	 */
	uint16_t interval;

	/**
	 * @brief Timeout (in milliseconds) for the entire location request.
	 *
	 * @details SYS_FOREVER_MS means that the timer is disabled.
	 *
	 * Default value is 300000 (5 minutes). It is applied when
	 * location_config_defaults_set() function is called and can be changed
	 * at build time with @kconfig{CONFIG_LOCATION_REQUEST_DEFAULT_TIMEOUT} configuration.
	 *
	 * This is intended to be a safety timer preventing location request from getting stuck
	 * in any of its phases, because parts of the method-specific functionality are not
	 * covered by method-specific timeouts. You should use a rather large value that
	 * can be one minute or more larger than the sum of method-specific timeouts.
	 */
	int32_t timeout;

	/**
	 * @brief Location acquisition mode.
	 *
	 * @details Default value is @ref LOCATION_REQ_MODE_FALLBACK. It is applied when
	 * location_config_defaults_set() function is called.
	 */
	enum location_req_mode mode;
};

/**
 * @brief Event handler prototype.
 *
 * @param[in] event_data Event data.
 */
typedef void (*location_event_handler_t)(const struct location_event_data *event_data);

/**
 * Register handler for location events.
 *
 * This function is not needed for clients that call location_init() and pass event handler into it.
 *
 * Multiple event handlers can be registered.
 *
 * @param[in] handler Event handler.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EINVAL The handler was a @c NULL pointer.
 * @retval -ENOMEM Out of memory.
 */
int location_handler_register(location_event_handler_t handler);

/**
 * Deregister handler for location events.
 *
 * @param[in] handler Event handler.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EINVAL The handler was not found or it was a @c NULL pointer.
 */
int location_handler_deregister(location_event_handler_t handler);

/**
 * @brief Initializes the library.
 *
 * @details Initializes the library and sets the event handler function.
 * The first call to this function must be called before going to LTE normal mode.
 * This can be called multiple times, which sets the event handler again.
 *
 * @param[in] handler Event handler function.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EINVAL Given event_handler is NULL.
 * @retval -EFAULT Failed to obtain Wi-Fi interface.
 */
int location_init(location_event_handler_t handler);

/**
 * @brief Requests the current position or starts periodic position updates.
 *
 * @details Requests the current position using the given configuration. Depending on the
 * configuration, a single position or periodic position updates are given. If the position
 * request is successful, the results are delivered to the event handler function in
 * @ref LOCATION_EVT_LOCATION event. Otherwise @ref LOCATION_EVT_TIMEOUT or
 * @ref LOCATION_EVT_ERROR event is triggered.
 *
 * In periodic mode, position update is always attempted after the configured interval
 * regardless of the outcome of any previous attempt. Periodic position updates can be
 * stopped by calling location_cancel().
 *
 * @param[in] config Used configuration or NULL to get a single position update with
 *                   the default configuration. Default configuration has the following
 *                   location methods in priority order (if they are enabled in library
 *                   configuration): GNSS, Wi-Fi and Cellular.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EPERM  Library not initialized.
 * @retval -EBUSY  Previous location request still ongoing.
 * @retval -EINVAL Invalid configuration given.
 */
int location_request(const struct location_config *config);

/**
 * @brief Cancels periodic position updates.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EPERM  Library not initialized.
 */
int location_request_cancel(void);

/**
 * @brief Sets the default values to a given configuration.
 *
 * @param[in,out] config Configuration to be supplied with default values.
 * @param[in] methods_count Number of location methods. This must not exceed
 *                          @kconfig{CONFIG_LOCATION_METHODS_LIST_SIZE}.
 * @param[in] method_types List of location method types in priority order.
 *                         Location methods with these types are initialized with default values
 *                         into given configuration. List size must be as given in 'methods_count'.
 */
void location_config_defaults_set(
	struct location_config *config,
	uint8_t methods_count,
	enum location_method *method_types);

/**
 * @brief Return location method as a string.
 *
 * @param[in] method Location method.
 *
 * @return Location method in string format. Returns "Unknown" if given method is not known.
 */
const char *location_method_str(enum location_method method);

/**
 * @brief Get location data details from the location event data.
 *
 * @details The @ref location_data_details structure is located in a different place in the
 * @ref location_event_data structure depending on the event ID. This is a helper function
 * to provide the @ref location_data_details structure.
 *
 * @param[in] event_data Event data.
 *
 * @return Location data details. NULL if there is no @ref location_data_details structure
 *         for the provided event.
 */
const struct location_data_details *location_details_get(
	const struct location_event_data *event_data);

/**
 * @brief Feed in A-GNSS data to be processed by library.
 *
 * @details If Location library is not receiving A-GNSS assistance data directly from nRF Cloud,
 * it triggers the @ref LOCATION_EVT_GNSS_ASSISTANCE_REQUEST event when assistance is needed. Once
 * application has obtained the assistance data it can be handed over to Location library using this
 * function.
 *
 * Note that the data must be formatted similarly to the nRF Cloud A-GNSS data, see for example
 * nrf_cloud_agnss_schema_v1.h.
 *
 * @param[in] buf Data received.
 * @param[in] buf_len Buffer size of data to be processed.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EINVAL Given buffer is NULL or buffer length zero.
 * @retval -ENOTSUP @kconfig{CONFIG_LOCATION_SERVICE_EXTERNAL} is not set.
 */
int location_agnss_data_process(const char *buf, size_t buf_len);

/**
 * @brief Feed in P-GPS data to be processed by library.
 *
 * @details If Location library is not receiving P-GPS assistance data directly from nRF Cloud,
 * it triggers the @ref LOCATION_EVT_GNSS_PREDICTION_REQUEST event when assistance is needed. Once
 * application has obtained the assistance data it can be handed over to Location library using this
 * function.
 *
 * Note that the data must be formatted similarly to the nRF Cloud P-GPS data, see for example
 * nrf_cloud_pgps_schema_v1.h.
 *
 * @param[in] buf Data received.
 * @param[in] buf_len Buffer size of data to be processed.
 *
 * @return 0 on success, or negative error code on failure.
 * @retval -EINVAL Given buffer is NULL or buffer length zero.
 * @retval -ENOTSUP @kconfig{CONFIG_LOCATION_SERVICE_EXTERNAL} is not set.
 */
int location_pgps_data_process(const char *buf, size_t buf_len);

/**
 * @brief Pass cloud location result to the library.
 *
 * @details If the Location library is not receiving cloud location directly from the services,
 * it triggers the @ref LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST event, that indicates the
 * LTE neighbor cell and/or Wi-Fi access point information is ready to be sent to cloud services
 * for location resolution. Then, the application responds with the result.
 *
 * In addition to 'success' and 'error' results, the application can indicate that the result is
 * 'unknown' with @ref LOCATION_EXT_RESULT_UNKNOWN. This is useful when the application wants
 * the Location library to proceed irrespective of the outcome. The Location library will try to
 * perform a fallback to the next method, if available, just like in a failure case.
 * If there are no more methods, @ref LOCATION_EVT_RESULT_UNKNOWN event will be sent to the
 * application.
 *
 * @param[in] result Result of the external cloud location request.
 * @param[in] location Cloud location data. Will be used only if @p result is
 *                     @ref LOCATION_EXT_RESULT_SUCCESS.
 */
void location_cloud_location_ext_result_set(
	enum location_ext_result result,
	struct location_data *location);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* LOCATION_H_ */
