/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @brief Command IDs accepted by NFC over RPC client.
 */
enum nfc_rpc_cmd_client {
	NFC_RPC_CMD_T2T_CB,
	NFC_RPC_CMD_T4T_CB,
};

/** @brief Command IDs accepted by NFC over RPC server.
 */
enum nfc_rpc_cmd_server {
	NFC_RPC_CMD_T2T_SETUP,
	NFC_RPC_CMD_T2T_PARAMETER_SET,
	NFC_RPC_CMD_T2T_PARAMETER_GET,
	NFC_RPC_CMD_T2T_PAYLOAD_SET,
	NFC_RPC_CMD_T2T_PAYLOAD_RAW_SET,
	NFC_RPC_CMD_T2T_INTERNAL_SET,
	NFC_RPC_CMD_T2T_EMULATION_START,
	NFC_RPC_CMD_T2T_EMULATION_STOP,
	NFC_RPC_CMD_T2T_DONE,
	NFC_RPC_CMD_T4T_SETUP,
	NFC_RPC_CMD_T4T_NDEF_RWPAYLOAD_SET,
	NFC_RPC_CMD_T4T_NDEF_STATICPAYLOAD_SET,
	NFC_RPC_CMD_T4T_RESPONSE_PDU_SEND,
	NFC_RPC_CMD_T4T_PARAMETER_SET,
	NFC_RPC_CMD_T4T_PARAMETER_GET,
	NFC_RPC_CMD_T4T_EMULATION_START,
	NFC_RPC_CMD_T4T_EMULATION_STOP,
	NFC_RPC_CMD_T4T_DONE,
};
