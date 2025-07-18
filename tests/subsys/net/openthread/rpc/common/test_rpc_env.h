/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>

/* Macros for constructing CBOR data items. */

#define CBOR_FALSE 0xf4
#define CBOR_TRUE  0xf5
#define CBOR_NULL  0xf6

#define CBOR_UINT64(value) 0x1B, BT_BYTES_LIST_LE64(BSWAP_64(value))
#define CBOR_UINT32(value) 0x1A, BT_BYTES_LIST_LE32(BSWAP_32(value))
#define CBOR_UINT16(value) 0x19, BT_BYTES_LIST_LE16(BSWAP_16(value))
/* for value bigger than 0x17 */
#define CBOR_UINT8(value)  0x18, (value)

#define CBOR_LIST(...) 0x9F, __VA_ARGS__, 0xFF
#define CBOR_EMPTY_BYTES 0x40
#define CBOR_EMPTY_STRING 0x60

/* Macros for constructing nRF RPC packets for the OpenThread command group. */

#define RPC_PKT(bytes...)                                                                          \
	(mock_nrf_rpc_pkt_t)                                                                       \
	{                                                                                          \
		.data = (uint8_t[]){bytes}, .len = sizeof((uint8_t[]){bytes}),                     \
	}

#define RPC_INIT_REQ         RPC_PKT(0x04, 0x00, 0xff, 0x00, 0xff, 0x00, 'o', 't')
#define RPC_INIT_RSP         RPC_PKT(0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 'o', 't')
#define RPC_CMD(cmd, ...)    RPC_PKT(0x80, cmd, 0xff, 0x00, 0x00 __VA_OPT__(,) __VA_ARGS__, 0xf6)
#define RPC_CB_CMD(cmd, ...) RPC_PKT(0x80, cmd, 0x00, 0x00, 0x00 __VA_OPT__(,) __VA_ARGS__, 0xf6)
#define RPC_RSP(...)         RPC_PKT(0x01, 0xff, 0x00, 0x00, 0x00 __VA_OPT__(,) __VA_ARGS__, 0xf6)
#define RPC_ERR(...)	     RPC_PKT(__VA_ARGS__)
#define NO_RSP               RPC_PKT()

/* Dummy test data */

#define LSFY_IDENTITY(n, _)  n + 1
#define INT_SEQUENCE(length) LISTIFY(length, LSFY_IDENTITY, (,))

#define LSFY_IDENTITY64(n, _) 0xFFFFFFFFFFFFFFFF - n
#define INT64_SEQUENCE(length) LISTIFY(length, LSFY_IDENTITY64, (,))

#define CBOR_LSFY_IDENTITY64(n, _)  0x1B, BT_BYTES_LIST_LE64(BSWAP_64(0xFFFFFFFFFFFFFFFF - n))
#define CBOR_INT64_SEQUENCE(length) LISTIFY(length, CBOR_LSFY_IDENTITY64, (,))

#define LSFY_CHAR(n, _)	     'a'
#define STR_SEQUENCE(length) LISTIFY(length, LSFY_CHAR, (,))

#define ADDR_1                                                                                     \
	0xfe, 0x80, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa,  \
		0x01
#define ADDR_2                                                                                     \
	0xfe, 0x80, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa, 0xbb, 0xaa,  \
		0x02
#define ADDR_PREFIX_LENGTH 64
#define ADDR_SCOPE	   15
#define MESH_LOCAL_PREFIX  INT_SEQUENCE(OT_MESH_LOCAL_PREFIX_SIZE)
#define PORT_1		   0xff01
#define PORT_2		   0xff02
#define HOP_LIMIT	   64
#define LINK_QUALITY	   3
#define THREAD_VERSION	   5
#define RLOC16		   0x1234
#define RESOURCE_TABLE_KEY 1
#define DNS_NAME                                                                                   \
	STR_SEQUENCE(63), '.', STR_SEQUENCE(63), '.', STR_SEQUENCE(63), '.', STR_SEQUENCE(63)
#define VERSION_STR STR_SEQUENCE(64)
#define MADDR_FF02_1  0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01
#define MADDR_FF02_2  0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02
#define EXT_ADDR       0x48, INT_SEQUENCE(OT_EXT_ADDRESS_SIZE)
#define NWK_NAME       0x70, INT_SEQUENCE(OT_NETWORK_NAME_MAX_SIZE)
#define EXT_PAN_ID     0x48, INT_SEQUENCE(OT_EXT_PAN_ID_SIZE)
#define STEERING_DATA  0x50, INT_SEQUENCE(OT_STEERING_DATA_MAX_LENGTH)
#define TLVS           0x58, 0xFE, INT_SEQUENCE(OT_OPERATIONAL_DATASET_MAX_LENGTH)
#define NWK_KEY        0x50, INT_SEQUENCE(OT_NETWORK_KEY_SIZE)
#define LOCAL_PREFIX   0x48, INT_SEQUENCE(OT_IP6_PREFIX_SIZE)
#define PSKC           0x50, INT_SEQUENCE(OT_PSKC_MAX_SIZE)
#define TIMESTAMP     CBOR_UINT64(0x0123456789abcdef), CBOR_UINT16(0x1234), CBOR_FALSE
#define SECURITY_POLICY                                                                            \
	CBOR_UINT16(0x9876), CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE,  \
		CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, 0x07
#define DATASET_COMPONENTS                                                                         \
	CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE,           \
		CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE
#define DATASET                                                                                    \
	TIMESTAMP, TIMESTAMP, NWK_KEY, NWK_NAME, EXT_PAN_ID, LOCAL_PREFIX,                         \
		CBOR_UINT32(0x12345678), CBOR_UINT16(0xabcd), CBOR_UINT16(0xef67), PSKC,           \
		SECURITY_POLICY, CBOR_UINT32(0xfedcba98), DATASET_COMPONENTS
#define CBOR_MSG_INFO                                                                              \
	0x50, ADDR_1, 0x50, ADDR_2, CBOR_UINT16(PORT_1), CBOR_UINT16(PORT_2), CBOR_UINT8(64), 3,   \
		CBOR_TRUE, CBOR_TRUE, CBOR_TRUE
#define CBOR_DNS_NAME 0x78, 0xff, DNS_NAME
#define CBOR_ADDR1 0x50, ADDR_1
#define CBOR_ADDR2 0x50, ADDR_2
#define CBOR_MESH_LOCAL_PREFIX 0x48, MESH_LOCAL_PREFIX

#define CBOR_SOC_ADDR CBOR_ADDR1, CBOR_UINT16(1024)

#define MESHDIAG_ROUTER_ID	    1
#define MESHDIAG_LINK_QUALITIES_LEN 63
#define MESHDIAG_LINK_QUALITIES	    0x58, 0x3F, INT_SEQUENCE(MESHDIAG_LINK_QUALITIES_LEN)
#define MESHDIAG_DISCOVER_CONFIG    0x3
#define MESHDIAG_ROUTER_INFO_FLAGS  0xF
#define MESHDIAG_CHILD_INFO_FLAGS   CBOR_UINT8(0x1F)

#define NETDATA_LENGTH	       254
#define NETDATA		       INT_SEQUENCE(NETDATA_LENGTH)
#define NETDATA_SVC_ID	       0x51
#define NETDATA_SVC_ENTERPRISE 0x51012345
#define NETDATA_SVC_DATA       INT_SEQUENCE(OT_SERVICE_DATA_MAX_SIZE)
#define NETDATA_SVR_DATA       INT_SEQUENCE(OT_SERVER_DATA_MAX_SIZE)
#define NETDATA_SVR_RLOC       0x1234
#define NETDATA_BR_RLOC	       0x2345
#define CBOR_NETDATA	       0x58, NETDATA_LENGTH, NETDATA
#define CBOR_NETDATA_SVC                                                                           \
	CBOR_UINT8(NETDATA_SVC_ID), CBOR_UINT32(NETDATA_SVC_ENTERPRISE), 0x58,                     \
		OT_SERVICE_DATA_MAX_SIZE, NETDATA_SVC_DATA, CBOR_TRUE, 0x58,                       \
		OT_SERVER_DATA_MAX_SIZE, NETDATA_SVR_DATA, CBOR_UINT16(NETDATA_SVR_RLOC)
#define CBOR_NETDATA_BR                                                                            \
	CBOR_ADDR1, CBOR_UINT8(64), 0x20, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, \
		CBOR_FALSE, CBOR_TRUE, CBOR_FALSE, CBOR_TRUE, CBOR_UINT16(NETDATA_BR_RLOC)

#define LSFY_NETDIAG(n, _) 0, 1, 2, 7
#define NETDIAG_SEQUENCE(length) LISTIFY(length, LSFY_NETDIAG, (,))

#define NETDIAG_CHILD_TABLE_LEN 63

#define NETDIAG_TLVS_LEN	    35
#define NETDIAG_TLVS		    INT_SEQUENCE(NETDIAG_TLVS_LEN)
#define NETDIAG_NEXT_ITERATOR	    1
#define NETDIAG_VENDOR_DATA_16      {STR_SEQUENCE(16), '\0'}
#define NETDIAG_VENDOR_DATA_17      {STR_SEQUENCE(17), '\0'}
#define NETDIAG_VENDOR_DATA_32      {STR_SEQUENCE(32), '\0'}
#define NETDIAG_VENDOR_DATA_33      {STR_SEQUENCE(33), '\0'}
#define CBOR_NETDIAG_VENDOR_DATA_16 0x70, STR_SEQUENCE(16)
#define CBOR_NETDIAG_VENDOR_DATA_17 0x71, STR_SEQUENCE(17)
#define CBOR_NETDIAG_VENDOR_DATA_32 0x78, 0x20, STR_SEQUENCE(32)
#define CBOR_NETDIAG_VENDOR_DATA_33 0x78, 0x21, STR_SEQUENCE(33)
#define CBOR_NETDIAG_TLVS	    0x58, 0x23, NETDIAG_TLVS

#define NETDIAG_TLV_EXT_ADDR		 INT_SEQUENCE(OT_EXT_ADDRESS_SIZE)
#define NETDIAG_TLV_EUI64		 INT_SEQUENCE(OT_EXT_ADDRESS_SIZE)
#define NETDIAG_TLV_CONNECTIVITY	 INT_SEQUENCE(9)
#define NETDIAG_TLV_ROUTE		 0, 63, NETDIAG_SEQUENCE(63)
#define NETDIAG_TLV_LEADER_DATA		 INT_SEQUENCE(5)
#define NETDIAG_TLV_MAC_COUNTERS	 INT_SEQUENCE(9)
#define NETDIAG_TLV_MLE_COUNTERS	 INT_SEQUENCE(9), INT64_SEQUENCE(6)
#define NETDIAG_TLV_BATTERY_LEVEL	 100
#define NETDIAG_TLV_TIMEOUT              0xFFFFFFFF
#define NETDIAG_TLV_MAX_CHILD_TIMEOUT    0xFFFFFFFF
#define NETDIAG_TLV_SHORT_ADDRESS        RLOC16
#define NETDIAG_TLV_SUPPLY_VOLTAGE	 5
#define NETDIAG_TLV_VERSION		 THREAD_VERSION
#define NETDIAG_TLV_VENDOR_NAME		 STR_SEQUENCE(32)
#define NETDIAG_TLV_VENDOR_MODEL	 STR_SEQUENCE(32)
#define NETDIAG_TLV_VENDOR_SW_VERSION	 STR_SEQUENCE(16)
#define NETDIAG_TLV_THREAD_STACK_VERSION STR_SEQUENCE(64)
#define NETDIAG_TLV_VENDOR_APP_URL	 STR_SEQUENCE(96)
#define NETDIAG_TLV_NETWORK_DATA	 INT_SEQUENCE(OT_NETWORK_BASE_TLV_MAX_LENGTH)
#define NETDIAG_TLV_CHANNEL_PAGES	 INT_SEQUENCE(OT_NETWORK_BASE_TLV_MAX_LENGTH)
#define NETDIAG_TLV_CHILD_TABLE		 NETDIAG_SEQUENCE(NETDIAG_CHILD_TABLE_LEN)

#define CBOR_NETDIAG_TLV_EXT_ADDR	      OT_NETWORK_DIAGNOSTIC_TLV_EXT_ADDRESS, EXT_ADDR
#define CBOR_NETDIAG_TLV_EUI64		      OT_NETWORK_DIAGNOSTIC_TLV_EUI64, EXT_ADDR
#define CBOR_NETDIAG_TLV_MODE		      OT_NETWORK_DIAGNOSTIC_TLV_MODE, 0x7
#define CBOR_NETDIAG_TLV_CONNECTIVITY	      OT_NETWORK_DIAGNOSTIC_TLV_CONNECTIVITY,		   \
					      NETDIAG_TLV_CONNECTIVITY
#define CBOR_NETDIAG_TLV_ROUTE		      OT_NETWORK_DIAGNOSTIC_TLV_ROUTE, 0, CBOR_UINT8(63),  \
					      CBOR_LIST(NETDIAG_SEQUENCE(63))
#define CBOR_NETDIAG_TLV_LEADER_DATA	      OT_NETWORK_DIAGNOSTIC_TLV_LEADER_DATA,		   \
					      NETDIAG_TLV_LEADER_DATA
#define CBOR_NETDIAG_TLV_MAC_COUNTERS	      OT_NETWORK_DIAGNOSTIC_TLV_MAC_COUNTERS,		   \
					      NETDIAG_TLV_MAC_COUNTERS
#define CBOR_NETDIAG_TLV_MLE_COUNTERS	      CBOR_UINT8(OT_NETWORK_DIAGNOSTIC_TLV_MLE_COUNTERS),  \
					      INT_SEQUENCE(9), CBOR_INT64_SEQUENCE(6)
#define CBOR_NETDIAG_TLV_BATTERY_LEVEL	      OT_NETWORK_DIAGNOSTIC_TLV_BATTERY_LEVEL,		   \
					      CBOR_UINT8(NETDIAG_TLV_BATTERY_LEVEL)
#define CBOR_NETDIAG_TLV_TIMEOUT	      OT_NETWORK_DIAGNOSTIC_TLV_TIMEOUT,		   \
					      CBOR_UINT32(NETDIAG_TLV_TIMEOUT)
#define CBOR_NETDIAG_TLV_MAX_CHILD_TIMEOUT    OT_NETWORK_DIAGNOSTIC_TLV_MAX_CHILD_TIMEOUT,	   \
					      CBOR_UINT32(NETDIAG_TLV_MAX_CHILD_TIMEOUT)
#define CBOR_NETDIAG_TLV_SHORT_ADDRESS	      OT_NETWORK_DIAGNOSTIC_TLV_SHORT_ADDRESS,		   \
					      CBOR_UINT16(NETDIAG_TLV_SHORT_ADDRESS)
#define CBOR_NETDIAG_TLV_SUPPLY_VOLTAGE	      OT_NETWORK_DIAGNOSTIC_TLV_SUPPLY_VOLTAGE,		   \
					      NETDIAG_TLV_SUPPLY_VOLTAGE
#define CBOR_NETDIAG_TLV_VERSION	      CBOR_UINT8(OT_NETWORK_DIAGNOSTIC_TLV_VERSION),	   \
					      NETDIAG_TLV_VERSION
#define CBOR_NETDIAG_TLV_VENDOR_NAME	      CBOR_UINT8(OT_NETWORK_DIAGNOSTIC_TLV_VENDOR_NAME),   \
					      0x78, 0x20, NETDIAG_TLV_VENDOR_NAME
#define CBOR_NETDIAG_TLV_VENDOR_MODEL	      CBOR_UINT8(OT_NETWORK_DIAGNOSTIC_TLV_VENDOR_MODEL),  \
					      0x78, 0x20, NETDIAG_TLV_VENDOR_MODEL
#define CBOR_NETDIAG_TLV_VENDOR_SW_VERSION    CBOR_UINT8(					   \
						OT_NETWORK_DIAGNOSTIC_TLV_VENDOR_SW_VERSION),	   \
					      0x70, NETDIAG_TLV_VENDOR_SW_VERSION
#define CBOR_NETDIAG_TLV_THREAD_STACK_VERSION CBOR_UINT8(					   \
						OT_NETWORK_DIAGNOSTIC_TLV_THREAD_STACK_VERSION),   \
					      0x78, 0x40, NETDIAG_TLV_THREAD_STACK_VERSION
#define CBOR_NETDIAG_TLV_VENDOR_APP_URL	      CBOR_UINT8(OT_NETWORK_DIAGNOSTIC_TLV_VENDOR_APP_URL),\
					      0x78, 0x60, NETDIAG_TLV_VENDOR_APP_URL
#define CBOR_NETDIAG_TLV_NETWORK_DATA	      OT_NETWORK_DIAGNOSTIC_TLV_NETWORK_DATA,		   \
					      CBOR_UINT8(OT_NETWORK_BASE_TLV_MAX_LENGTH), 0x58,    \
					      0xFE, NETDIAG_TLV_NETWORK_DATA
#define CBOR_NETDIAG_TLV_CHANNEL_PAGES	      OT_NETWORK_DIAGNOSTIC_TLV_CHANNEL_PAGES,		   \
					      CBOR_UINT8(OT_NETWORK_BASE_TLV_MAX_LENGTH), 0x58,    \
					      0xFE, NETDIAG_TLV_CHANNEL_PAGES
#define CBOR_NETDIAG_TLV_IP6_ADDR_LIST	      OT_NETWORK_DIAGNOSTIC_TLV_IP6_ADDR_LIST, 2,	   \
					      CBOR_LIST(CBOR_ADDR1, CBOR_ADDR2)
#define CBOR_NETDIAG_TLV_CHILD_TABLE	      OT_NETWORK_DIAGNOSTIC_TLV_CHILD_TABLE,		   \
					      CBOR_UINT8(NETDIAG_CHILD_TABLE_LEN),		   \
					      CBOR_LIST(NETDIAG_SEQUENCE(NETDIAG_CHILD_TABLE_LEN))

#define DNS_HOSTNAME "example.com"
#define DNS_SERVICE_NAME "_test._udp.example.com"
#define DNS_SERVICE_INSTANCE "Test"
#define DNS_IPV6_ADDR 0x20, 0x01, 0xd, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1
#define DNS_PORT 53
#define DNS_TIMEOUT 1000
#define DNS_ATTEMPTS 5

#define CBOR_IPV6_ADDR 0x50, DNS_IPV6_ADDR
#define CBOR_DNS_QUERY_CONFIG CBOR_IPV6_ADDR, CBOR_UINT8(DNS_PORT), CBOR_UINT16(DNS_TIMEOUT), \
			      DNS_ATTEMPTS, OT_DNS_FLAG_RECURSION_DESIRED, OT_DNS_NAT64_DISALLOW, \
			      OT_DNS_SERVICE_MODE_SRV_TXT_OPTIMIZE, OT_DNS_TRANSPORT_UDP

#define CBOR_DNS_HOSTNAME 0x6B, 'e', 'x', 'a', 'm', 'p', 'l', 'e', '.', 'c', 'o', 'm'
#define CBOR_DNS_SERVICE_NAME 0x76, '_', 't', 'e', 's', 't', '.', '_', 'u', 'd', 'p', '.', 'e', \
			      'x', 'a', 'm', 'p', 'l', 'e', '.', 'c', 'o', 'm'
#define CBOR_SERVICE_INSTANCE 0x64, 'T', 'e', 's', 't'
#define CBOR_TXT_DATA 0x50, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x20, 0x54, 0x58, 0x54, \
		      0x20, 0x44, 0x41, 0x54, 0x41 /* "Service TXT DATA" */

#define CBOR_TTL CBOR_UINT8(120)
#define CBOR_PORT CBOR_UINT16(12345)
#define CBOR_PRIORITY 10
#define CBOR_WEIGHT 12

#define CBOR_SERVICE_INFO CBOR_TTL, CBOR_PORT, CBOR_PRIORITY, CBOR_WEIGHT, CBOR_DNS_HOSTNAME, \
			  CBOR_IPV6_ADDR, CBOR_TTL, CBOR_TXT_DATA, CBOR_FALSE, CBOR_TTL
