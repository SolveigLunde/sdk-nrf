/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/coap.h>
#if defined(CONFIG_POSIX_API)
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/sys/socket.h>
#else
#include <zephyr/net/socket.h>
#endif
#include <modem/lte_lc.h>
#if defined(CONFIG_MODEM_KEY_MGMT)
#include <modem/modem_key_mgmt.h>
#endif
#if defined(CONFIG_MODEM_INFO)
#include <modem/modem_info.h>
#endif
#if defined(CONFIG_NRF_MODEM_LIB)
#include <nrf_socket.h>
#include <nrf_modem_at.h>
#endif
#include <net/nrf_cloud.h>

#include "nrfc_dtls.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dtls, CONFIG_NRF_CLOUD_COAP_LOG_LEVEL);

static bool dtls_cid_active;
static bool cid_supported = true;
static bool keepopen_supported;

static int get_device_ip_address(uint8_t *d4_addr)
{
#if defined(CONFIG_MODEM_INFO)
	char buf[INET_ADDRSTRLEN + sizeof(" ") + INET6_ADDRSTRLEN + 1];
	int err;

	err = modem_info_string_get(MODEM_INFO_IP_ADDRESS, buf, sizeof(buf));
	if (err <= 0) {
		LOG_ERR("Could not get IP addr: %d", err);
		return err;
	}
	err = zsock_inet_pton(AF_INET, buf, d4_addr);
	if (err == 1) {
		return 0;
	}
	return -errno;
#else
	d4_addr[0] = 0;
	d4_addr[1] = 0;
	d4_addr[2] = 0;
	d4_addr[3] = 0;
	return 0;
#endif

}

int nrfc_dtls_setup(int sock)
{
	int err;
	int sectag;
	uint8_t d4_addr[4];

	/* once connected, cache the connection info */
	dtls_cid_active = false;

	err = get_device_ip_address(d4_addr);
	if (!err) {
		LOG_DBG("Client IP address: %u.%u.%u.%u",
			d4_addr[0], d4_addr[1], d4_addr[2], d4_addr[3]);
	}

	LOG_DBG("Setting socket options:");

	LOG_DBG("  hostname: %s", CONFIG_NRF_CLOUD_COAP_SERVER_HOSTNAME);
	err = zsock_setsockopt(sock, SOL_TLS, TLS_HOSTNAME, CONFIG_NRF_CLOUD_COAP_SERVER_HOSTNAME,
			 sizeof(CONFIG_NRF_CLOUD_COAP_SERVER_HOSTNAME));
	if (err) {
		LOG_ERR("Error setting hostname: %d", -errno);
		return -errno;
	}

	sectag = nrf_cloud_sec_tag_get();
	LOG_DBG("  sectag: %d", sectag);

	err = zsock_setsockopt(sock, SOL_TLS, TLS_SEC_TAG_LIST, &sectag, sizeof(sectag));
	if (err) {
		LOG_ERR("Error setting sectag list: %d", -errno);
		return -errno;
	}

	int cid_option = TLS_DTLS_CID_SUPPORTED;

	LOG_DBG("  Enable connection id");
	err = zsock_setsockopt(sock, SOL_TLS, TLS_DTLS_CID, &cid_option, sizeof(cid_option));
	if (!err) {
		cid_supported = true;
	} else if ((err != EOPNOTSUPP) && (err != EINVAL)) {
		LOG_ERR("Error enabling connection ID: %d", -errno);
		cid_supported = false;
	} else {
		LOG_INF("Connection ID not supported by the provided socket");
		cid_supported = false;
	}

#if !defined(CONFIG_BOARD_NATIVE_SIM)
	int timeout = TLS_DTLS_HANDSHAKE_TIMEO_123S;

	LOG_DBG("  Set handshake timeout %d", timeout);
	err = zsock_setsockopt(sock, SOL_TLS, TLS_DTLS_HANDSHAKE_TIMEO,
			 &timeout, sizeof(timeout));
	if (!err) {
	} else if ((err != EOPNOTSUPP) || (err != EINVAL)) {
		LOG_ERR("Error setting handshake timeout: %d", -errno);
	}
#endif

	int verify = TLS_PEER_VERIFY_REQUIRED;

	LOG_DBG("  Peer verify: %d", verify);
	err = zsock_setsockopt(sock, SOL_TLS, TLS_PEER_VERIFY, &verify, sizeof(verify));
	if (err) {
		LOG_ERR("Failed to setup peer verification, errno %d", -errno);
		return -errno;
	}

	int session_cache = TLS_SESSION_CACHE_ENABLED;

	LOG_DBG("  TLS session cache: %d", session_cache);
	err = zsock_setsockopt(sock, SOL_TLS, TLS_SESSION_CACHE,
			       &session_cache, sizeof(session_cache));
	if (err) {
		LOG_ERR("Failed to enable session cache, errno: %d", -errno);
		err = -errno;
	}

	keepopen_supported = false;
	if (IS_ENABLED(CONFIG_NRF_CLOUD_COAP_KEEPOPEN)) {
		err = zsock_setsockopt(sock, SOL_SOCKET, SO_KEEPOPEN, &(int){1}, sizeof(int));
		if (err) {
			/* Either not supported or unusable due to unknown error. */
			err = 0;
		} else {
			keepopen_supported = true;
		}
	}
	LOG_DBG("  Keep open supported: %d", keepopen_supported);
	return err;
}

int nrfc_dtls_session_save(int sock)
{
	int err;

	LOG_DBG("Save DTLS CID session");
	err = zsock_setsockopt(sock, SOL_TLS, TLS_DTLS_CONN_SAVE, &(int){0}, sizeof(int));
	if (err) {
		LOG_DBG("Failed to save DTLS CID session, errno %d", -errno);
		err = -errno;
	}
	return err;
}

int nrfc_dtls_session_load(int sock)
{
	int err;

	LOG_DBG("Load DTLS CID session");
	err = zsock_setsockopt(sock, SOL_TLS, TLS_DTLS_CONN_LOAD, &(int){1}, sizeof(int));
	if (err) {
		LOG_DBG("Failed to load DTLS CID session, errno %d", -errno);
		err = -errno;
	}
	return err;
}

bool nrfc_dtls_cid_is_active(int sock)
{
	int err = 0;

	if (dtls_cid_active) {
		return true;
	}

	int status = 0;
	int len = sizeof(status);

#if defined(CONFIG_NRF_CLOUD_COAP_LOG_LEVEL_DBG)

	err = zsock_getsockopt(sock, SOL_TLS, TLS_DTLS_HANDSHAKE_STATUS, &status, &len);
	if (!err) {
		if (len > 0) {
			if (status == TLS_DTLS_HANDSHAKE_STATUS_FULL) {
				LOG_DBG("Full DTLS handshake performed");
			} else if (status == TLS_DTLS_HANDSHAKE_STATUS_CACHED) {
				LOG_DBG("Cached DTLS handshake performed");
			} else {
				LOG_WRN("Unknown DTLS handshake status: %d", status);
			}
		} else {
			LOG_WRN("No DTLS status provided");
		}
	} else if ((errno != EOPNOTSUPP) && (errno != EINVAL)) {
		LOG_ERR("Error retrieving handshake status: %d", -errno);
	} /* else the current modem firmware does not support this feature */

#endif /* CONFIG_NRF_CLOUD_COAP_LOG_LEVEL_DBG */

	len = sizeof(status);
	err = zsock_getsockopt(sock, SOL_TLS, TLS_DTLS_CID_STATUS, &status, &len);
	if (!err) {
		if (len > 0) {
			switch (status) {
			case TLS_DTLS_CID_STATUS_DISABLED:
				dtls_cid_active = false;
				LOG_DBG("No DTLS CID used");
				break;
			case TLS_DTLS_CID_STATUS_DOWNLINK:
				dtls_cid_active = false;
				LOG_DBG("DTLS CID downlink");
				break;
			case TLS_DTLS_CID_STATUS_UPLINK:
				dtls_cid_active = true;
				LOG_DBG("DTLS CID uplink");
				break;
			case TLS_DTLS_CID_STATUS_BIDIRECTIONAL:
				dtls_cid_active = true;
				LOG_DBG("DTLS CID bidirectional");
				break;
			default:
				LOG_WRN("Unknown DTLS CID status: %d", status);
				break;
			}
		} else {
			LOG_WRN("No DTLS CID status provided");
		}
	} else {
		LOG_ERR("Error retrieving DTLS CID status: %d", -errno);
	}

	return dtls_cid_active;
}

bool nrfc_keepopen_is_supported(void)
{
	return keepopen_supported;
}
