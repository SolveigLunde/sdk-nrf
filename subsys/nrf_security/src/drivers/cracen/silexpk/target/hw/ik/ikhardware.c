/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cracen/membarriers.h>

#include "ikhardware.h"
#include "../ba414/op_slots.h"
#include "../ik/regs_commands.h"
#include "../ba414/regs_addr.h"
#include "../ba414/pkhardware_ba414e.h"
#include "../ba414/ba414_status.h"
#include <silexpk/iomem.h>
#include <cracen/statuscodes.h>
#include "../../include/sx_regs.h"
#include "regs_addr.h"
#include <silexpk/ec_curves.h>
#include <silexpk/ik.h>
#ifdef CONFIG_CRACEN_HW_VERSION_LITE
#include "hal/nrf_cracen.h"
#endif

int cracen_prepare_ik_key(const uint8_t *user_data);

int sx_pk_is_ik_cmd(sx_pk_req *req)
{
	return req->cmd->cmdcode & SX_PK_OP_IK;
}

int sx_ik_read_status(sx_pk_req *req)
{
	uint32_t status = sx_pk_rdreg(&req->regs, IK_REG_PK_STATUS);

	if (status & 1) {
		return SX_ERR_NOT_IMPLEMENTED;
	}
	if (status & 2) {
		return SX_ERR_UNKNOWN_ERROR;
	}
	status = sx_pk_rdreg(&req->regs, PK_REG_STATUS);
	return convert_ba414_status(status);
}

int sx_pk_list_ik_inslots(sx_pk_req *req, unsigned int key, struct sx_pk_slot *inputs)
{
	int slots = req->cmd->inslots;
	char *cryptoram = req->cryptoram;
	int i = 0;
	const struct sx_pk_capabilities *caps;

	if (req->cmd->cmdcode == PK_OP_IK_EXIT) {
#ifdef CONFIG_CRACEN_HW_VERSION_LITE
		/* Workaround to handle IKG freezing on CRACEN lite.
		 * THE PKE-IKG interrupt can not be cleared from software, but can
		 * be cleared by hardware when in PK mode.
		 * so the interrupt is disabled after leaving PK mode and enabled when entering.
		 */
		nrf_cracen_int_disable(NRF_CRACEN, CRACEN_INTENCLR_PKEIKG_Msk);
#endif
		req->ik_mode = 0;
	} else {
		if (!req->ik_mode) {
			int status = cracen_prepare_ik_key((uint8_t *)&key);
			if (status != SX_OK) {
				sx_pk_release_req(req);
				return SX_ERR_INVALID_PARAM;
			}
			sx_pk_wrreg(&req->regs, PK_REG_CONTROL, PK_RB_CONTROL_CLEAR_IRQ);
			req->ik_mode = 1;
		}
	}

	caps = sx_pk_fetch_capabilities();

	if (!caps->ik_opsz) {
		sx_pk_release_req(req);
		return SX_ERR_IK_NOT_READY;
	}

	int slot_sz = caps->max_gfp_opsz;

	req->op_size = caps->ik_opsz;
	uint32_t rval = req->cmd->cmdcode & 0x301;

	/* Write IK cmd register */
	sx_pk_wrreg(&req->regs, IK_REG_PK_COMMAND, rval);

	if (req->cmd->cmdcode & SX_PK_OP_FLAGS_BIGENDIAN) {
		/* In big endian mode, the operands should be put at the end
		 * of the slot.
		 */
		cryptoram += slot_sz - req->op_size;
	}
	while (slots) {
		if (slots & 1) {
			inputs[i++].addr = cryptoram;
		}
		cryptoram += slot_sz;
		slots >>= 1;
	}

	return 0;
}

static int sx_ik_gen_keys(struct sx_regs *regs, struct sx_pk_config_ik *cfg)
{
	uint32_t status = 0;
	int error = 0;
	int success = 0;

	/* Reset */
	sx_pk_wrreg(regs, IK_REG_SOFT_RST, 1);
	sx_pk_wrreg(regs, IK_REG_SOFT_RST, 0);

	/* Wait for CTRDRB health check */
	status = IK_CTRDRBG_BUSY;
	while (status & IK_CTRDRBG_BUSY) {
		status = sx_pk_rdreg(regs, IK_REG_STATUS);
	}
	if (status & IK_CATASTROPHIC_ERR) {
		return SX_ERR_UNKNOWN_ERROR;
	}

	uint32_t config = sx_pk_rdreg(regs, IK_REG_HW_CONFIG);

	sx_pk_wrreg(regs, IK_REG_INITDATA, 1);
	int person_sz = (config >> 24) & 0xF;

	/** Write key bundle **/
	if (cfg && cfg->key_bundle && cfg->key_bundle_sz) {
		if (cfg->key_bundle_sz > person_sz) {
			return SX_ERR_OPERAND_TOO_LARGE;
		}

		for (int i = 0; i < cfg->key_bundle_sz; i++) {
			sx_pk_wrreg(regs, IK_REG_PERSONALIZATION_STRING, cfg->key_bundle[i]);
		}
	}
	/** Maximal device secret length **/
	int nonce_sz;
	int reg_addr;

	if (config & (1 << 12)) { /* DF enabled */
		reg_addr = IK_REG_NONCE;
		nonce_sz = (config >> 20) & 0xF;
	} else {
		reg_addr = IK_REG_PERSONALIZATION_STRING;
		nonce_sz = person_sz;
		if (cfg && cfg->key_bundle_sz) {
			nonce_sz -= cfg->key_bundle_sz;
		}
	}
	/** Write device secret **/
	if (cfg && cfg->device_secret && cfg->device_secret_sz) {
		if (cfg->device_secret_sz > nonce_sz) {
			return SX_ERR_OPERAND_TOO_LARGE;
		}

		for (int i = 0; i < cfg->device_secret_sz; i++) {
			sx_pk_wrreg(regs, reg_addr, cfg->device_secret[i]);
		}
	}

	/* Start key generation */
	sx_pk_wrreg(regs, IK_REG_START, 1);
	while (!success && !error) {
		status = sx_pk_rdreg(regs, IK_REG_STATUS);
		error = (status & 0x3) | (status & IK_CATASTROPHIC_ERR);
		/* Only check the okay flag. If checking the
		 * priv_key_stored flag only, the CTR_DRBG routine can still
		 * be executing while the keys present (= priv_key_stored flag
		 * set).
		 */
		success = ((status >> 2) & 1);
	}

	if (success) {
		return 0;
	}

	return SX_ERR_UNKNOWN_ERROR;
}

void sx_pk_read_ik_capabilities(struct sx_regs *regs, struct sx_pk_capabilities *caps)
{
	uint32_t config = sx_pk_rdreg(regs, IK_REG_HW_CONFIG);

	config &= IK_CURVE_MASK;
	int ik_opsz = 0;

	/* Get curve ID */
	switch (config >> 10) {
	case 0:
		ik_opsz = 32;
		break;
	case 1:
		ik_opsz = 48;
		break;
	case 2:
		ik_opsz = 66;
		break;
	}
	caps->ik_opsz = ik_opsz;
}

int sx_pk_ik_derive_keys(struct sx_pk_config_ik *cfg)
{
	struct sx_regs *regs = sx_pk_get_regs();
	int r = sx_ik_gen_keys(regs, cfg);

	if (!r) {
		struct sx_pk_capabilities *caps = sx_pk_get_caps();

		sx_pk_read_ik_capabilities(regs, caps);
	}

	return r;
}

int sx_pk_ik_rng_reconfig(struct sx_pk_cnx *cnx, struct sx_pk_config_rng *cfg)
{
	if (cfg->personalization && cfg->personalization_sz) {
		struct sx_regs *regs = sx_pk_get_regs();
		uint32_t config = sx_pk_rdreg(regs, IK_REG_HW_CONFIG);
		int max_perso_sz = (config >> 24) & 0xF;

		if (cfg->personalization_sz > max_perso_sz) {
			return SX_ERR_OPERAND_TOO_LARGE;
		}

		/* Write Personalization string */
		sx_pk_wrreg(regs, IK_REG_INITDATA, 1);
		for (int i = 0; i < cfg->personalization_sz; i++) {
			sx_pk_wrreg(regs, IK_REG_PERSONALIZATION_STRING, cfg->personalization[i]);
		}
	}

	return SX_OK;
}
