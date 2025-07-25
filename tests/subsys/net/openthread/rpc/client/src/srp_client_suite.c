/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "mocks.h"

#include <mock_nrf_rpc_transport.h>
#include <ot_rpc_ids.h>
#include <ot_rpc_types.h>
#include <test_rpc_env.h>

#include <zephyr/fff.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <openthread/srp_client.h>

/* Common test data */

#define SERVICE_TYPE	  '_', 't', 'y', 'p', 'e'
#define SERVICE_INSTANCE  'i', 'n', 's', 't'
#define SERVICE_SUBTYPE1  '_', 'S', '1'
#define SERVICE_SUBTYPE2  '_', 'S', '2'
#define SERVICE_TXT1_KEY  'T', '1'
#define SERVICE_TXT1_VAL  0xff, 0xff, 0xff, 0x01
#define SERVICE_TXT2_KEY  'T', '2'
#define SERVICE_TXT2_VAL  0xff, 0xff, 0xff, 0x02
#define SERVICE_PRIORITY  UINT16_MAX
#define SERVICE_WEIGHT	  (UINT16_MAX - 1)
#define SERVICE_LEASE	  UINT32_MAX
#define SERVICE_KEY_LEASE (UINT32_MAX - 1)

#define _SERVER_IPV6_START 0x20, 0x01, 0x0D, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
			   0x00, 0x00, 0x00

#define SERVER_IPV6_ADDR1_BYTES _SERVER_IPV6_START, 0x1
#define SERVER_IPV6_ADDR2_BYTES _SERVER_IPV6_START, 0x2
#define SERVER_IPV6_ADDR3_BYTES _SERVER_IPV6_START, 0x3
#define SERVER_IPV6_ADDR4_BYTES _SERVER_IPV6_START, 0x4

#define CBOR_SERVER_IPV6_ADDR1 0x50, SERVER_IPV6_ADDR1_BYTES
#define CBOR_SERVER_IPV6_ADDR2 0x50, SERVER_IPV6_ADDR2_BYTES
#define CBOR_SERVER_IPV6_ADDR3 0x50, SERVER_IPV6_ADDR3_BYTES
#define CBOR_SERVER_IPV6_ADDR4 0x50, SERVER_IPV6_ADDR4_BYTES

#define SERVER_PORT 12345

/*
 * NOTE:
 * The following service encoding is correct assuming that zcbor encodes arrays and maps
 * with indefinite lengths, which is true unless ZCBOR_CANONICAL Kconfig is not selected.
 */

/* clang-format off */
#define CBOR_SERVICE_MIN \
	/* Subtypes num: */ \
	0x0, \
	/* TXT entry count: */ \
	0x0, \
	/* String buffer size (service + instance): */ \
	0xb, \
	/* Subtypes' buffer size: */ \
	0x0, \
	/* TXT buffer size: */ \
	0x0, \
	/* Service type: */ \
	0x65, SERVICE_TYPE, \
	/* Service instance: */ \
	0x64, SERVICE_INSTANCE, \
	/* Subtypes: */ \
	0x9f, \
	0xff, \
	/* TXT: */ \
	0xbf, \
	0xff, \
	/* Other fields: */ \
	CBOR_UINT16(PORT_1), \
	CBOR_UINT16(SERVICE_PRIORITY), \
	CBOR_UINT16(SERVICE_WEIGHT), \
	CBOR_UINT32(SERVICE_LEASE), \
	CBOR_UINT32(SERVICE_KEY_LEASE)
/* clang-format on */

/* clang-format off */
#define CBOR_SERVICE \
	/* Subtypes num: */ \
	0x2, \
	/* TXT entry count: */ \
	0x2, \
	/* String buffer size (service + instance): */ \
	0xb, \
	/* Subtypes' buffer size: */ \
	0xa, \
	/* TXT buffer size: */ \
	0x14, \
	/* Service type: */ \
	0x65, SERVICE_TYPE, \
	/* Service instance: */ \
	0x64, SERVICE_INSTANCE, \
	/* Subtypes: */ \
	0x9f, \
		0x63, SERVICE_SUBTYPE1, \
		0x63, SERVICE_SUBTYPE2, \
	0xff, \
	/* TXT: */ \
	0xbf, \
		0x62, SERVICE_TXT1_KEY, 0x44, SERVICE_TXT1_VAL, \
		0x62, SERVICE_TXT2_KEY, 0x44, SERVICE_TXT2_VAL, \
	0xff, \
	/* Other fields: */ \
	CBOR_UINT16(PORT_1), \
	CBOR_UINT16(SERVICE_PRIORITY), \
	CBOR_UINT16(SERVICE_WEIGHT), \
	CBOR_UINT32(SERVICE_LEASE), \
	CBOR_UINT32(SERVICE_KEY_LEASE)
/* clang-format on */

#define MAKE_CSTR(...)	     ((char[]){__VA_ARGS__ __VA_OPT__(,) '\0'})
#define MAKE_BYTE_ARRAY(...) ((uint8_t[]){__VA_ARGS__})

/* Fake functions */

FAKE_VOID_FUNC(ot_srp_client_auto_start_cb, const otSockAddr *, void *);
FAKE_VOID_FUNC(ot_srp_client_cb, otError, const otSrpClientHostInfo *, const otSrpClientService *,
	       const otSrpClientService *, void *);

static void nrf_rpc_err_handler(const struct nrf_rpc_err_report *report)
{
	zassert_ok(report->code);
}

static void *tc_setup(void)
{
	mock_nrf_rpc_tr_expect_add(RPC_INIT_REQ, RPC_INIT_RSP);
	zassert_ok(nrf_rpc_init(nrf_rpc_err_handler));
	mock_nrf_rpc_tr_expect_reset();

	return NULL;
}

static void tc_after(void *f)
{
	/* Clear all host and service data after each test case */
	mock_nrf_rpc_tr_expect_reset();
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_CLEAR_HOST_AND_SERVICES),
				   RPC_RSP());
	otSrpClientClearHostAndServices(NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientAddService() with the minimal service data */
ZTEST(ot_rpc_srp_client, test_otSrpClientAddService_min)
{
	otError error;
	otSrpClientService service;

	service.mName = MAKE_CSTR(SERVICE_TYPE);
	service.mInstanceName = MAKE_CSTR(SERVICE_INSTANCE);
	service.mSubTypeLabels = NULL;
	service.mTxtEntries = NULL;
	service.mPort = PORT_1;
	service.mPriority = SERVICE_PRIORITY;
	service.mWeight = SERVICE_WEIGHT;
	service.mNumTxtEntries = 0;
	service.mLease = SERVICE_LEASE;
	service.mKeyLease = SERVICE_KEY_LEASE;

	/* Test serialization of otSrpClientAddService() */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ADD_SERVICE,
					   CBOR_UINT32((uintptr_t)&service), CBOR_SERVICE_MIN),
				   RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientAddService(NULL, &service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);
}

/* Test serialization of otSrpClientAddService() followed by otSrpClientClearService() */
ZTEST(ot_rpc_srp_client, test_otSrpClientAddService_otSrpClientClearService)
{
	const char * const sub_types[] = {
		MAKE_CSTR(SERVICE_SUBTYPE1),
		MAKE_CSTR(SERVICE_SUBTYPE2),
		NULL,
	};

	const otDnsTxtEntry txt_entries[] = {
		{
			.mKey = MAKE_CSTR(SERVICE_TXT1_KEY),
			.mValue = MAKE_BYTE_ARRAY(SERVICE_TXT1_VAL),
			.mValueLength = sizeof(MAKE_BYTE_ARRAY(SERVICE_TXT1_VAL)),
		},
		{
			.mKey = MAKE_CSTR(SERVICE_TXT2_KEY),
			.mValue = MAKE_BYTE_ARRAY(SERVICE_TXT2_VAL),
			.mValueLength = sizeof(MAKE_BYTE_ARRAY(SERVICE_TXT2_VAL)),
		},
	};

	otError error;
	otSrpClientService service;

	service.mName = MAKE_CSTR(SERVICE_TYPE);
	service.mInstanceName = MAKE_CSTR(SERVICE_INSTANCE);
	service.mSubTypeLabels = sub_types;
	service.mTxtEntries = txt_entries;
	service.mPort = PORT_1;
	service.mPriority = SERVICE_PRIORITY;
	service.mWeight = SERVICE_WEIGHT;
	service.mNumTxtEntries = ARRAY_SIZE(txt_entries);
	service.mLease = SERVICE_LEASE;
	service.mKeyLease = SERVICE_KEY_LEASE;

	/* Test serialization of otSrpClientAddService() */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ADD_SERVICE,
					   CBOR_UINT32((uintptr_t)&service), CBOR_SERVICE),
				   RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientAddService(NULL, &service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);

	/* Test serialization of otSrpClientClearService() */
	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_CLEAR_SERVICE, CBOR_UINT32((uintptr_t)&service)),
		RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientClearService(NULL, &service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);

	/* Verify that clearing the service that has not been registered results in error */
	error = otSrpClientClearService(NULL, &service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NOT_FOUND);
}

static void verify_ot_srp_client_cb_locked(otError error, const otSrpClientHostInfo *host,
					   const otSrpClientService *services,
					   const otSrpClientService *removed, void *context)
{
	ARG_UNUSED(error);
	ARG_UNUSED(host);
	ARG_UNUSED(services);
	ARG_UNUSED(removed);
	ARG_UNUSED(context);

	zassert_true(ot_rpc_is_mutex_locked());
}

/* Test serialization of otSrpClientAddService() and SRP client callback */
ZTEST(ot_rpc_srp_client, test_otSrpClientAddService_callback)
{
	const char * const sub_types[] = {
		MAKE_CSTR(SERVICE_SUBTYPE1),
		MAKE_CSTR(SERVICE_SUBTYPE2),
		NULL,
	};

	const otDnsTxtEntry txt_entries[] = {
		{
			.mKey = MAKE_CSTR(SERVICE_TXT1_KEY),
			.mValue = MAKE_BYTE_ARRAY(SERVICE_TXT1_VAL),
			.mValueLength = sizeof(MAKE_BYTE_ARRAY(SERVICE_TXT1_VAL)),
		},
		{
			.mKey = MAKE_CSTR(SERVICE_TXT2_KEY),
			.mValue = MAKE_BYTE_ARRAY(SERVICE_TXT2_VAL),
			.mValueLength = sizeof(MAKE_BYTE_ARRAY(SERVICE_TXT2_VAL)),
		},
	};

	otError error;
	otSrpClientService service;

	service.mName = MAKE_CSTR(SERVICE_TYPE);
	service.mInstanceName = MAKE_CSTR(SERVICE_INSTANCE);
	service.mSubTypeLabels = sub_types;
	service.mTxtEntries = txt_entries;
	service.mPort = PORT_1;
	service.mPriority = SERVICE_PRIORITY;
	service.mWeight = SERVICE_WEIGHT;
	service.mNumTxtEntries = ARRAY_SIZE(txt_entries);
	service.mLease = SERVICE_LEASE;
	service.mKeyLease = SERVICE_KEY_LEASE;

	/* Test serialization of otSrpClientAddService() */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ADD_SERVICE,
					   CBOR_UINT32((uintptr_t)&service), CBOR_SERVICE),
				   RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientAddService(NULL, &service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);

	/* Test serialization of otSrpClientSetCallback() that takes non-null callback */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_CALLBACK, CBOR_TRUE),
				   RPC_RSP());
	otSrpClientSetCallback(NULL, ot_srp_client_cb, (void *)UINT32_MAX);
	mock_nrf_rpc_tr_expect_done();

	/* Test remote call of the client callback that reports registered host & service */
	RESET_FAKE(ot_srp_client_cb);
	ot_srp_client_cb_fake.custom_fake = verify_ot_srp_client_cb_locked;

	mock_nrf_rpc_tr_expect_add(RPC_RSP(), NO_RSP);
	mock_nrf_rpc_tr_receive(RPC_CMD(
		OT_RPC_CMD_SRP_CLIENT_CB, OT_ERROR_NONE, OT_SRP_CLIENT_ITEM_STATE_REGISTERED,
		CBOR_UINT32((uintptr_t)&service), OT_SRP_CLIENT_ITEM_STATE_REGISTERED));
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(ot_srp_client_cb_fake.call_count, 1);
	zexpect_equal(ot_srp_client_cb_fake.arg0_val, OT_ERROR_NONE);
	zassert_not_null(ot_srp_client_cb_fake.arg1_val);
	zexpect_equal(ot_srp_client_cb_fake.arg1_val->mState, OT_SRP_CLIENT_ITEM_STATE_REGISTERED);
	zexpect_equal(ot_srp_client_cb_fake.arg2_val, &service);
	zexpect_equal(ot_srp_client_cb_fake.arg2_val->mState, OT_SRP_CLIENT_ITEM_STATE_REGISTERED);
	zexpect_is_null(ot_srp_client_cb_fake.arg2_val->mNext);
	zexpect_is_null(ot_srp_client_cb_fake.arg3_val);

	/* Test remote call of the client callback that reports removed service */
	RESET_FAKE(ot_srp_client_cb);
	ot_srp_client_cb_fake.custom_fake = verify_ot_srp_client_cb_locked;

	mock_nrf_rpc_tr_expect_add(RPC_RSP(), NO_RSP);
	mock_nrf_rpc_tr_receive(RPC_CMD(
		OT_RPC_CMD_SRP_CLIENT_CB, OT_ERROR_NONE, OT_SRP_CLIENT_ITEM_STATE_REGISTERED,
		CBOR_UINT32((uintptr_t)&service), OT_SRP_CLIENT_ITEM_STATE_REMOVED));
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(ot_srp_client_cb_fake.call_count, 1);
	zexpect_equal(ot_srp_client_cb_fake.arg0_val, OT_ERROR_NONE);
	zassert_not_null(ot_srp_client_cb_fake.arg1_val);
	zexpect_equal(ot_srp_client_cb_fake.arg1_val->mState, OT_SRP_CLIENT_ITEM_STATE_REGISTERED);
	zexpect_is_null(ot_srp_client_cb_fake.arg2_val);
	zexpect_equal(ot_srp_client_cb_fake.arg3_val, &service);
	zexpect_equal(ot_srp_client_cb_fake.arg3_val->mState, OT_SRP_CLIENT_ITEM_STATE_REMOVED);
	zexpect_is_null(ot_srp_client_cb_fake.arg3_val->mNext);

	/* Test serialization of otSrpClientSetCallback() that takes null callback */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_CALLBACK, CBOR_FALSE),
				   RPC_RSP());
	otSrpClientSetCallback(NULL, NULL, NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientClearHostAndServices() */
ZTEST(ot_rpc_srp_client, test_otSrpClientClearHostAndServices)
{
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_CLEAR_HOST_AND_SERVICES),
				   RPC_RSP());
	otSrpClientClearHostAndServices(NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientDisableAutoStartMode() */
ZTEST(ot_rpc_srp_client, test_otSrpClientDisableAutoStartMode)
{
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_DISABLE_AUTO_START_MODE),
				   RPC_RSP());
	otSrpClientDisableAutoStartMode(NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientEnableAutoHostAddress() */
ZTEST(ot_rpc_srp_client, test_otSrpClientEnableAutoHostAddress)
{
	otError error;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ENABLE_AUTO_HOST_ADDR),
				   RPC_RSP(OT_ERROR_INVALID_STATE));
	error = otSrpClientEnableAutoHostAddress(NULL);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_INVALID_STATE);
}

static void ot_srp_client_auto_start_cb_custom(const otSockAddr *addr, void *context)
{
	zassert_true(ot_rpc_is_mutex_locked());
	zassert_not_null(addr);
	zexpect_mem_equal(addr->mAddress.mFields.m8, (uint8_t[]){ADDR_1}, OT_IP6_ADDRESS_SIZE);
	zexpect_equal(addr->mPort, PORT_1);
	zexpect_equal(context, (void *)UINT32_MAX);
}

/* Test serialization of otSrpClientEnableAutoStartMode() */
ZTEST(ot_rpc_srp_client, test_otSrpClientEnableAutoStartMode)
{
	/* Test serialization of otSrpClientEnableAutoStartMode() that takes non-null callback */
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ENABLE_AUTO_START_MODE, CBOR_TRUE),
				   RPC_RSP());
	otSrpClientEnableAutoStartMode(NULL, ot_srp_client_auto_start_cb, (void *)UINT32_MAX);
	mock_nrf_rpc_tr_expect_done();

	/* Test remote call of the auto start callback */
	RESET_FAKE(ot_srp_client_auto_start_cb);
	ot_srp_client_auto_start_cb_fake.custom_fake = ot_srp_client_auto_start_cb_custom;

	mock_nrf_rpc_tr_expect_add(RPC_RSP(), NO_RSP);
	mock_nrf_rpc_tr_receive(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_AUTO_START_CB, 0x50, ADDR_1, CBOR_UINT32(PORT_1)));
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(ot_srp_client_auto_start_cb_fake.call_count, 1);

	/* Test serialization of otSrpClientEnableAutoStartMode() that takes null callback
	 */
	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_ENABLE_AUTO_START_MODE, CBOR_FALSE), RPC_RSP());
	otSrpClientEnableAutoStartMode(NULL, NULL, NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientRemoveHostAndServices() */
ZTEST(ot_rpc_srp_client, test_otSrpClientRemoveHostAndServices)
{
	otError error;

	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_REMOVE_HOST_AND_SERVICES, CBOR_FALSE, CBOR_TRUE),
		RPC_RSP(OT_ERROR_INVALID_STATE));
	error = otSrpClientRemoveHostAndServices(NULL, false, true);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_INVALID_STATE);
}

/* Test serialization of otSrpClientRemoveService() */
ZTEST(ot_rpc_srp_client, test_otSrpClientRemoveService)
{
	otSrpClientService *service = (otSrpClientService *)UINT32_MAX;
	otError error;

	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_REMOVE_SERVICE, CBOR_UINT32(UINT32_MAX)),
		RPC_RSP(OT_ERROR_INVALID_STATE));
	error = otSrpClientRemoveService(NULL, service);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_INVALID_STATE);
}

/* Test serialization of otSrpClientSetHostName() */
ZTEST(ot_rpc_srp_client, test_otSrpClientSetHostName)
{
	static const char hostname[] = {DNS_NAME, '\0'};
	otError error;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_HOSTNAME, CBOR_DNS_NAME),
				   RPC_RSP(OT_ERROR_INVALID_STATE));
	error = otSrpClientSetHostName(NULL, hostname);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_INVALID_STATE);
}

/* Test serialization of otSrpClientSetKeyLeaseInterval() */
ZTEST(ot_rpc_srp_client, test_otSrpClientSetKeyLeaseInterval)
{
	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_KEY_LEASE_INTERVAL, CBOR_UINT32(UINT32_MAX)),
		RPC_RSP());
	otSrpClientSetKeyLeaseInterval(NULL, UINT32_MAX);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientSetLeaseInterval() */
ZTEST(ot_rpc_srp_client, test_otSrpClientSetLeaseInterval)
{
	mock_nrf_rpc_tr_expect_add(
		RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_LEASE_INTERVAL, CBOR_UINT32(UINT32_MAX)),
		RPC_RSP());
	otSrpClientSetLeaseInterval(NULL, UINT32_MAX);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientSetTtl() */
ZTEST(ot_rpc_srp_client, test_otSrpClientSetTtl)
{
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_TTL, CBOR_UINT32(UINT32_MAX)),
				   RPC_RSP());
	otSrpClientSetTtl(NULL, UINT32_MAX);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientStart() */
ZTEST(ot_rpc_srp_client, test_otSrpClientStart)
{
	otError error;
	otSockAddr sock_addr = {
		.mAddress = {{{ SERVER_IPV6_ADDR1_BYTES }}},
		.mPort = SERVER_PORT,
	};

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_START, CBOR_SERVER_IPV6_ADDR1,
				   CBOR_UINT16(SERVER_PORT)), RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientStart(NULL, &sock_addr);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);
}

/* Test serialization of otSrpClientStop() */
ZTEST(ot_rpc_srp_client, test_otSrpClientStop)
{
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_STOP), RPC_RSP());
	otSrpClientStop(NULL);
	mock_nrf_rpc_tr_expect_done();
}

/* Test serialization of otSrpClientIsRunning() */
ZTEST(ot_rpc_srp_client, test_otSrpClientIsRunning)
{
	bool err_1st;
	bool err_2nd;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_IS_RUNNING), RPC_RSP(CBOR_FALSE));
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_IS_RUNNING), RPC_RSP(CBOR_TRUE));

	err_1st = otSrpClientIsRunning(NULL);
	err_2nd = otSrpClientIsRunning(NULL);

	mock_nrf_rpc_tr_expect_done();

	zassert_false(err_1st);
	zassert_true(err_2nd);
}

/* Test serialization of otSrpClientGetServerAddress() */
ZTEST(ot_rpc_srp_client, test_otSrpClientGetServerAddress)
{
	const otSockAddr *sock_addr;

	otIp6Address expected_addr = {{{ SERVER_IPV6_ADDR1_BYTES }}};

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_GET_SERVER_ADDRESS),
				   RPC_RSP(CBOR_SERVER_IPV6_ADDR1, CBOR_UINT16(SERVER_PORT)));

	sock_addr = otSrpClientGetServerAddress(NULL);

	mock_nrf_rpc_tr_expect_done();

	zassert_mem_equal(&sock_addr->mAddress, &expected_addr, OT_IP6_ADDRESS_SIZE);
	zassert_equal(sock_addr->mPort, SERVER_PORT);
}

/* Test serialization of otSrpClientSetHostAddresses() */
ZTEST(ot_rpc_srp_client, test_otSrpClientSetHostAddresses)
{
	otError error;

	otIp6Address addrs[] = { {{{ SERVER_IPV6_ADDR1_BYTES }}}, {{{ SERVER_IPV6_ADDR2_BYTES }}},
				 {{{ SERVER_IPV6_ADDR3_BYTES }}}, {{{ SERVER_IPV6_ADDR4_BYTES }}} };

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_SET_HOST_ADDRESSES, 4,
				   CBOR_LIST(CBOR_SERVER_IPV6_ADDR1, CBOR_SERVER_IPV6_ADDR2,
					     CBOR_SERVER_IPV6_ADDR3, CBOR_SERVER_IPV6_ADDR4)),
				   RPC_RSP(OT_ERROR_NONE));
	error = otSrpClientSetHostAddresses(NULL, addrs, 4);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(error, OT_ERROR_NONE);
}

/* Test serialization of otSrpClientIsAutoStartModeEnabled() */
ZTEST(ot_rpc_srp_client, test_otSrpClientIsAutoStartModeEnabled)
{
	bool err1;
	bool err2;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_IS_AUTO_START_MODE_ENABLED),
				   RPC_RSP(CBOR_FALSE));
	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_IS_AUTO_START_MODE_ENABLED),
				   RPC_RSP(CBOR_TRUE));

	err1 = otSrpClientIsAutoStartModeEnabled(NULL);
	err2 = otSrpClientIsAutoStartModeEnabled(NULL);

	mock_nrf_rpc_tr_expect_done();

	zassert_false(err1);
	zassert_true(err2);
}

/* Test serialization of otSrpClientGetLeaseInterval() */
ZTEST(ot_rpc_srp_client, test_otSrpClientGetLeaseInterval)
{
	uint32_t interval;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_GET_LEASE_INTERVAL),
				   RPC_RSP(CBOR_UINT32(100000)));
	interval = otSrpClientGetLeaseInterval(NULL);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(interval, 100000);
}

/* Test serialization of otSrpClientGetTtl() */
ZTEST(ot_rpc_srp_client, test_otSrpClientGetTtl)
{
	uint32_t ttl;

	mock_nrf_rpc_tr_expect_add(RPC_CMD(OT_RPC_CMD_SRP_CLIENT_GET_TTL),
				   RPC_RSP(CBOR_UINT8(120)));
	ttl = otSrpClientGetTtl(NULL);
	mock_nrf_rpc_tr_expect_done();

	zassert_equal(ttl, 120);
}

ZTEST_SUITE(ot_rpc_srp_client, NULL, tc_setup, NULL, tc_after, NULL);
