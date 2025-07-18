/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief Gazell Pairing API
 */

#ifndef __GZP_H
#define __GZP_H

#include <stdint.h>
#include <stdbool.h>
#include <nrf_gzll.h>
#if defined(CONFIG_GAZELL_PAIRING_USER_CONFIG_ENABLE)
#include CONFIG_GAZELL_PAIRING_USER_CONFIG_FILE
#else
#include "gzp_config.h"
#endif /* CONFIG_GAZELL_PAIRING_USER_CONFIG_ENABLE */

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup gzp Gazell Pairing
 * @{
 */


/******************************************************************************/
/** @name Pairing configuration defines
 *  @{
 */
/******************************************************************************/

/** Pipe reserved for initial pairing communication. */
#define GZP_PAIRING_PIPE 0
/** Pipe reserved for GZP encrypted data communication (one pipe only). */
#define GZP_DATA_PIPE 1
/** Time to wait between request and fetch packets in RX_PERIODS (2 timeslot periods) */
#define GZP_TX_RX_TRANS_DELAY 10
/** Must equal Gazell base address length. */
#define GZP_SYSTEM_ADDRESS_WIDTH 4


/** Validation ID. Required to be shared by Host and Device. Not a secret. */
#define GZP_VALIDATION_ID {0x32, 0x53, 0x66}
/** Validation ID length in bytes. */
#define GZP_VALIDATION_ID_LENGTH 3
/** Host ID length in bytes. */
#define GZP_HOST_ID_LENGTH 5
/** Session token length in bytes. */
#define GZP_SESSION_TOKEN_LENGTH GZP_HOST_ID_LENGTH
/** Dynamic key length in bytes. */
#define GZP_DYN_KEY_LENGTH (16 - GZP_VALIDATION_ID_LENGTH)

/** RSSI threshold for when signal strength in RX packet power is high enough. */
#define GZP_HOST_RX_POWER_THRESHOLD -64

/** @} */


/******************************************************************************/
/** @name Device -> Host packet definitions
 *  @{
 */
/******************************************************************************/

/** "Host address request" packet, payload length */
#define GZP_CMD_HOST_ADDRESS_REQ_PAYLOAD_LENGTH 1

/** "Host address fetch" packet, payload length */
#define GZP_CMD_HOST_ADDRESS_FETCH_PAYLOAD_LENGTH 1

/** "Host ID request" packet, session token position */
#define GZP_CMD_HOST_ID_REQ_SESSION_TOKEN 1
/* "Host ID request" payload length */
#define GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_REQ_SESSION_TOKEN + \
					    GZP_SESSION_TOKEN_LENGTH)

#if (GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH > 17)
#error GZP_SESSION_TOKEN_LENGTH too long.
#endif

/** "Host ID fetch" packet */
#define GZP_CMD_HOST_ID_FETCH_VALIDATION_ID 1
/** "Host ID fetch" payload length */
#define GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_FETCH_VALIDATION_ID + \
					      GZP_VALIDATION_ID_LENGTH)

#if (GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH > 17)
#error GZP_VALIDATION_ID_LENGTH set too long.
#endif

/** "Key update prepare" payload length */
#define GZP_CMD_KEY_UPDATE_PREPARE_PAYLOAD_LENGTH 1

/** "Key update" packet, validation ID position */
#define GZP_CMD_KEY_UPDATE_VALIDATION_ID 1
/** "Key update" packet, new key position */
#define GZP_CMD_KEY_UPDATE_NEW_KEY (GZP_CMD_KEY_UPDATE_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)
/** "Key update" packet, payload length */
#define GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH (GZP_CMD_KEY_UPDATE_NEW_KEY + GZP_DYN_KEY_LENGTH)

#if (GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH > 17)
#error Sum (GZP_VALIDATION_ID_LENGTH + GZP_DYN_KEY_LENGTH) too high.
#endif


/** "Encrypted user data" packet, validation ID position */
#define GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID 1
/** "Encrypted user data" packet, user data position */
#define GZP_CMD_ENCRYPTED_USER_DATA_PAYLOAD (GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID + \
					     GZP_VALIDATION_ID_LENGTH)
/** "Encrypted user data" packet, packet overhead length */
#define GZP_ENCRYPTED_USER_DATA_PACKET_OVERHEAD (GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID + \
						 GZP_VALIDATION_ID_LENGTH)
/** "Encrypted user data" packet, max payload length */
#define GZP_ENCRYPTED_USER_DATA_MAX_LENGTH (17 - GZP_ENCRYPTED_USER_DATA_PACKET_OVERHEAD)

#if (GZP_MAX_FW_PAYLOAD_LENGTH < 17)
#error GZP_MAX_FW_PAYLOAD_LENGTH must be greater or equal to 17.
#endif

/** General "fetch response" packet, payload_length */
#define GZP_CMD_FETCH_RESP_PAYLOAD_LENGTH 1

/** @} */


/******************************************************************************/
/** @name Host -> Device packet definitions
 *  @{
 */
/******************************************************************************/

/** "Host address fetch" response packet, address position */
#define GZP_CMD_HOST_ADDRESS_RESP_ADDRESS 1
/** "Host address fetch" response packet, payload length */
#define GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH (GZP_CMD_HOST_ADDRESS_RESP_ADDRESS + \
						  GZP_SYSTEM_ADDRESS_WIDTH)

#if (GZP_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH)
#error GZP_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to \
	GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH.
#endif


/** "Host ID fetch" response packet, validation ID position */
#define GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID 1
/** "Host ID fetch" response packet, status position */
#define GZP_CMD_HOST_ID_FETCH_RESP_STATUS (GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID + \
					   GZP_VALIDATION_ID_LENGTH)
/** "Host ID fetch" response packet, Host ID position */
#define GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID (GZP_CMD_HOST_ID_FETCH_RESP_STATUS + 1)
/** "Host ID fetch" response packet, payload length */
#define GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID + \
						   GZP_HOST_ID_LENGTH)

#if (GZP_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH)
#error GZP_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to \
	GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH.
#endif


/** "Key update prepare" response packet, session token position */
#define GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN 1
/** "Key update prepare" response packet, payload length position */
#define GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH \
		(GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN + \
		 GZP_SESSION_TOKEN_LENGTH)

#if (GZP_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH)
#error GZP_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to \
	GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH.
#endif


/** "Encrypted user data" response packet, session token position */
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN 1
/** "Encrypted user data" response packet, validation ID position */
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID \
		(GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN + \
		 GZP_SESSION_TOKEN_LENGTH)
/** "Encrypted user data" response packet, payload length position */
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH \
		(GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID + \
		 GZP_VALIDATION_ID_LENGTH)

#if (GZP_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH)
#error GZP_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to \
	GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH.
#endif

#if (GZP_VALIDATION_ID_LENGTH > GZP_HOST_ID_LENGTH)
#error GZP_HOST_ID_LENGTH should be greater or equal to \
	GZP_VALIDATION_ID_LENGTH.
#endif

#if (GZP_SESSION_TOKEN_LENGTH != GZP_HOST_ID_LENGTH)
#error GZP_SESSION_TOKEN_LENGTH must equal GZP_HOST_ID_LENGTH.
#endif

/** @} */


/******************************************************************************/
/** @name Typedefs
 *  @{
 */
/******************************************************************************/


/**
 * @enum gzp_key_select
 * @brief Enumerator used for selecting the key to be used for encryption.
 */
enum gzp_key_select {
	/** "Secret key" only */
	GZP_ID_EXCHANGE,
	/** "Secret key" and "Host ID" */
	GZP_KEY_EXCHANGE,
	/** "Dynamic key" and "Host ID" */
	GZP_DATA_EXCHANGE
};


/**
 * @enum gzp_cmd
 * @brief Enumerator used in the first payload byte of each packet to
 * indicate the packet type.
 */
enum gzp_cmd {
	/** Host address request */
	GZP_CMD_HOST_ADDRESS_REQ = 0,
	/** Host address fetch */
	GZP_CMD_HOST_ADDRESS_FETCH,
	/** Host ID request */
	GZP_CMD_HOST_ID_REQ,
	/** Host ID fetch request */
	GZP_CMD_HOST_ID_FETCH,
	/** Key update prepare */
	GZP_CMD_KEY_UPDATE_PREPARE,
	/** Key update */
	GZP_CMD_KEY_UPDATE,
	/** Encrypted user data */
	GZP_CMD_ENCRYPTED_USER_DATA,
	/** Fetch response */
	GZP_CMD_FETCH_RESP,
	/** Host address response */
	GZP_CMD_HOST_ADDRESS_RESP,
	/** Host ID fetch response */
	GZP_CMD_HOST_ID_FETCH_RESP,
	/** Key update prepare */
	GZP_CMD_KEY_UPDATE_PREPARE_RESP,
	/** Encrypted user data response */
	GZP_CMD_ENCRYPTED_USER_DATA_RESP,
};


/**
 * @enum gzp_id_req_res
 * @brief Enumerator used to identify the state of the current
 * Host ID request.
 */
enum gzp_id_req_res {
	/** ID response pending */
	GZP_ID_RESP_PENDING,
	/** ID response granted */
	GZP_ID_RESP_GRANTED,
	/** ID response rejected */
	GZP_ID_RESP_REJECTED,
	/** ID response failed */
	GZP_ID_RESP_FAILED,
	/** Default value. No ID request has yet been received. */
	GZP_ID_RESP_NO_REQUEST
};


/**
 * Callback function for reporting packet transmit result
 *
 * @param success is the packet transmit result.
 * @param pipe is the pipe.
 * @param tx_info struct is the information about the packet transmission.
 */
typedef void (*gzp_tx_result_callback)(bool success,
				       uint32_t pipe,
				       const nrf_gzll_device_tx_info_t *tx_info);


/**
 * Callback function for reporting "system address" request result
 *
 * @param result corresponds to gzp_address_req_send() return value.
 * @param context pointer to the user context.
 */
typedef void (*gzp_address_req_callback)(bool result, void *context);


/**
 * Callback function for reporting Host ID request result
 *
 * @param result corresponds to gzp_id_req_send() return value.
 * @param context pointer to the user context.
 */
typedef void (*gzp_id_req_callback)(enum gzp_id_req_res result, void *context);


/**
 * Callback function for reporting encrypted user data transmit result
 *
 * @param result corresponds to gzp_crypt_data_send() return value.
 * @param context pointer to the user context.
 */
typedef void (*gzp_crypt_data_send_callback)(bool result, void *context);


/** @} */


/******************************************************************************/
/** @name Misc. function prototypes
 *  @{
 */
/******************************************************************************/

/**
 * Set the session token.
 *
 * @param token Pointer to the session token to set.
 */
void gzp_crypt_set_session_token(const uint8_t *token);


/**
 * Get the session token.
 *
 * @param dst_token Pointer to write the session token.
 */
void gzp_crypt_get_session_token(uint8_t *dst_token);


/**
 * Set the dynamic key.
 *
 * @param dyn_key Pointer to the dynamic key to set.
 */
void gzp_crypt_set_dyn_key(const uint8_t *dyn_key);


/**
 * Get the dynamic key.
 *
 * @param dst_key Pointer to write the dynamic key too.
 */
void gzp_crypt_get_dyn_key(uint8_t *dst_key);


/**
 * Set the Host ID.
 *
 * @param src Pointer to the Host ID to set.
 */
void gzp_set_host_id(const uint8_t *src);


/**
 * Get the Host ID.
 *
 * @param dst Pointer to write the Host ID to.
 */
void gzp_get_host_id(uint8_t *dst);


/**
 * Selecting what key-set that should be used when encrypting data
 * using gzp_crypt().
 *
 * @param key_select Key-set to use.
 */
void gzp_crypt_select_key(enum gzp_key_select key_select);


/**
 * Encypt / decrypt data.
 *
 * The current "session token" will be used as initialization vector (IV).
 * The AES key to be used is selected by gzp_crypt_select_key().
 * AES is a symmetric encryption scheme, this function can be used
 * to perform both encryption and decryption.
 *
 * @param dst Destination to write encrypted data to. Should be 16 bytes long.
 * @param src Source data to encrypt.
 * @param length Length in bytes of src.
 */
void gzp_crypt(uint8_t *dst, const uint8_t *src, uint8_t length);


/**
 * Compare the *src_id with a pre-defined validation ID.
 *
 * @param src_id Pointer to the source validation ID to compare to.
 *
 * @retval true If *src_id equals the pre-defined ID.
 * @retval false If *src_id does not equal the pre-defined ID.
 */
bool gzp_validate_id(const uint8_t *src_id);


/**
 * Add the pre-defined validation ID to dst_id.
 * GZP_VALIDATION_ID_LENGTH bytes will be added.
 *
 * @param dst_id Pointer to add the GZP validation ID to.
 */
void gzp_add_validation_id(uint8_t *dst_id);


/**
 * Generate random bytes.
 *
 * @param dst Destination to write the random bytes to.
 * @param n   Number of bytes to generate.
 */
void gzp_random_numbers_generate(uint8_t *dst, uint8_t n);


/**
 * Update the channel table and the system address.
 *
 * The channel table is updated to pseudo-random set generated using the
 * system address. The channel table still includes GZP_CHANNEL_MAX and
 * GZP_CHANNEL_MIN.
 * The system address is applied to base address 1 and therefore applies
 * to pipes 1-7.
 *
 * @param system_address Pointer to the system_address to set.
 *
 * @retval true  If the radio parameters were updated successfully.
 * @retval false If there was an error updated the radio parameters.
 */
bool gzp_update_radio_params(const uint8_t *system_address);


/**
 * Generate a set of channels from a 4 byte address.
 *
 * @param ch_dst Destination to write the channel set to. The channel set
 * includes GZP_CHANNEL_MAX and GZP_CHANNEL_MIN.
 * @param address Four byte address to generate the channel set from.
 * @param channel_set_size Size of the channel set to generate.
 */
void gzp_generate_channels(uint8_t *ch_dst, const uint8_t *address, uint8_t channel_set_size);


/**
 * Perform an XOR on two byte strings.
 *
 * @param dst Destination to write the result to. Should be of size length.
 * @param src
 * @param pad
 * @param length Number of bytes to perform the XOR operation on.
 */
void gzp_xor_cipher(uint8_t *dst, const uint8_t *src, const uint8_t *pad, uint8_t length);


/******************************************************************************/
/** @name Common Device and Host functions
 *  @{
 */
/******************************************************************************/


/**
 * Initialization function. This function initializes the Gazell Pairing Library.
 *
 * This function must be called before any of the other Gazell Pairing Library functions are
 * used and must be called @b after gzll_init() is called.
 */
void gzp_init(void);

/**
 * Function for erasing all pairing data.
 */
void gzp_erase_pairing_data(void);

/**
 * Disable Gazell and sleep while waiting for nrf_gzll_disabled callback.
 */
void nrf_gzp_disable_gzll(void);

/**
 * Function for cancelling an ongoing (pending) "Host ID request".
 *
 * After calling this function the "Host ID request" status will go to
 * "ID request Idle".
 */
void gzp_id_req_cancel(void);

/**
 * Flush the GZLL RX FIFO for a specific pipe while GZLL is disabled.
 *
 * @param pipe Pipe.
 */
void nrf_gzp_flush_rx_fifo(uint32_t pipe);

/**
 * @name Device functions
 */

/**
 * Function for registering callback to be called on transmit done.
 *
 * @param callback is the callback function.
 */
void gzp_tx_result_callback_register(gzp_tx_result_callback callback);

/**
 * Function to check whether a Device has existing pairing data, implying that it is
 * paired to a Host.
 *
 * @retval -2 The pairing database is empty.
 * @retval -1 The device has a system address but no Host ID.
 * @retval >=0 The device has a system address and Host ID.
 */
int8_t gzp_get_pairing_status(void);

/**
 * Function for sending a "system address" request to a Host.
 *
 * When calling this function the Device will attempt acquiring the "system address" from
 * any Host within close proximity.
 *
 * If a host is located within close proximity and pairing is enabled in the Host,
 * a "system address" will be sent in return to the Device.
 *
 * The new "system address" will apply immediately in the Device, and the new "system address"
 * will be stored in non volatile (NV) memory.
 *
 * @retval true if new "system address" was received from a Host.
 * @retval false if no "system address" was received from a Host.
 */
bool gzp_address_req_send(void);

/**
 * Function for sending a "system address" request to a Host. Asynchronized fashion.
 *
 * @param callback is the callback function.
 * @param context is a pointer to be passed to the callback function.
 */
void gzp_address_req_send_async(gzp_address_req_callback callback, void *context);

/**
 * Function for sending a "Host ID request" to a Host.
 *
 * The "Host ID" is needed to be able to send encrypted data using
 * gzp_crypt_data_send().
 *
 * The request will be sent using the "system address" previously received using
 * gzp_address_req_send().
 *
 * It is not required that the Host is within close proximity in order to acquire the
 * "Host ID".
 *
 * The new "Host ID" will apply immediately for the Device, and the new "Host ID"
 * will be stored in non volatile (NV) memory.
 *
 * @retval GZP_ID_RESP_PENDING if a "Host ID request" has been sent to the Host, but the Host
 *                             application has not yet decided whether to Grant or Reject the
 *                             "ID request".
 * @retval GZP_ID_RESP_GRANTED if the "Host ID" has been received from the Host. The received
 *                             "Host ID" will be stored in non volatile memory.
 * @retval GZP_ID_RESP_REJECTED if the Host application has rejected the "Host ID request".
 * @retval GZP_ID_RESP_FAILED if failing to send a request or receive a response from the Host.
 */
enum gzp_id_req_res gzp_id_req_send(void);

/**
 * Function for sending a "Host ID request" to a Host. Asynchronized fashion.
 *
 * @param callback is the callback function.
 * @param context is a pointer to be passed to the callback function.
 */
void gzp_id_req_send_async(gzp_id_req_callback callback, void *context);

/**
 * Function for sending encrypted user data to the Host.
 *
 * Before any data can be sent the Device must acquire both the Host's
 * "system address" by using gzp_address_req_send() and the "Host ID" by using
 * gzp_id_req_send().
 *
 * @param src is a pointer to the data packet to be sent.
 * @param length is the length of the data packet to be sent.
 *
 * @retval true if the data was successfully transmitted and decrypted by the Host.
 * @retval false if data transmission failed or Host failed to decryption data correctly.
 */
bool gzp_crypt_data_send(const uint8_t *src, uint8_t length);

/**
 * Function for sending encrypted user data to the Host. Asynchronized fashion.
 *
 * @param src is a pointer to the data packet to be sent.
 * @param length is the length of the data packet to be sent.
 * @param callback is the callback function.
 * @param context is a pointer to be passed to the callback function.
 */
void gzp_crypt_data_send_async(const uint8_t *src,
			       uint8_t length,
			       gzp_crypt_data_send_callback callback,
			       void *context);


/**
 * @name Host functions
 */

/**
 * Function for enabling/disabling pairing in a host. When pairing is enabled the host will
 * be monitoring for "system address" and "Host ID" requests from Devices.
 *
 * A "system address request" received from a Device will always be granted.
 * When a "host ID request" has been received, the Host application have to grant,
 * reject or cancel this by using one of the following functions:
 *
 * - gzp_id_req_grant()
 * - gzp_id_req_reject()
 * - gzp_id_req_cancel()
 *
 * @param enable
 * @arg true enables pairing.
 * @arg false disables pairing.
 */
void gzp_pairing_enable(bool enable);

/**
 * Execute the Gazell Pairing Library Host operation.
 *
 * This function must be called regularly by the Host application.
 */
void gzp_host_execute(void);

/**
 * Address exchanged check.
 *
 * @retval true  If a "system address" was delivered to a requesting Device during the
 *               previous call to gzp_host_execute();
 * @retval false Otherwise.
 */
bool gzp_address_exchanged(void);

/**
 * Function for checking if a "Host ID request" has been received from a Device.
 *
 * If a request has been received, the Pairing library will enter "ID request pending"
 * state.
 *
 * The application is responsible for responding to this request by calling
 * one of the following functions:
 *
 * - gzp_id_req_grant()
 * - gzp_id_req_reject()
 * - gzp_id_req_cancel()
 *
 * @retval true if a "Host ID request" has been received (internal state is "ID request pending")
 * @retval false if no "Host ID request" has been received (internal state is "ID request idle")
 */
bool gzp_id_req_received(void);

/**
 * Function for rejecting the previously received "Host ID request". This function should be called
 * only when a "Host ID request" has been received (internal state is "ID request pending").
 *
 * The internal state of the Pairing library will remain "ID request pending" until the a "reject"
 * message has been successfully transmitted to the requesting Device. After this the internal
 * state will change to "ID request idle".
 */
void gzp_id_req_reject(void);

/**
 * Function for granting the previously received "Host ID request". This function should be called
 * only when a "Host ID request" has been received (internal state is "ID request pending").
 *
 * The internal state of the Pairing library will remain "ID request pending" until the "Host ID"
 * has been successfully transmitted to the requesting Device. After this the internal state will
 * change to "ID request idle".
 */
void gzp_id_req_grant(void);


/**
 * Check if user data has been received.
 *
 * @retval true  If encrypted user data has been received.
 * @retval false Otherwise.
 */
bool gzp_crypt_user_data_received(void);

/**
 * Function for reading encrypted user data.
 *
 * Note that the read user data will be automatically decrypted. Only data
 * that was decrypted correctly will be presented.
 *
 * @param dst is a pointer to where the received data will be written.
 * @param length is a pointer for returning the number of bytes received. Only 1 byte will
 * be writtem to length*.
 *
 * @retval true if data has been received and is written to dst*
 * @retval false if no data has been received.
 */
bool gzp_crypt_user_data_read(uint8_t *dst, uint8_t *length);


/**
 * Function emulating behavior of gzll_rx_start() in legeacy nRF24xx Gazell
 * library.
 *
 * This function sets Gazell in Host mode and starts reception (enable).
 */
void gzll_rx_start(void);


/** @} */
/** @} */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GZP_H */
