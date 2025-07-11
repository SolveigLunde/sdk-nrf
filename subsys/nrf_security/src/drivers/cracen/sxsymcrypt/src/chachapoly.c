/*
 *  Copyright (c) 2023 Nordic Semiconductor ASA
 *
 *  SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "../include/sxsymcrypt/aead.h"
#include "../include/sxsymcrypt/keyref.h"
#include <cracen/statuscodes.h>
#include "keyrefdefs.h"
#include "aeaddefs.h"
#include "blkcipherdefs.h"
#include "crypmasterregs.h"
#include "hw.h"
#include "cmdma.h"
#include "cmaes.h"
#include <string.h>

#define SX_CHACHAPOLY_NONCE_SZ		 12u
#define SX_CHACHAPOLY_COUNTER_SIZE	 4u
#define SX_CHACHAPOLY_KEY_SZ		 32u
#define SX_CHACHAPOLY_TAG_SIZE		 16u
#define SX_CHACHAPOLY_CTX_GRANULARITY_SZ 64u
#define SX_CHACHAPOLY_BLOCK_SZ 64u

/** Size of ChaCha20Poly1305 context saving state, in bytes
 * The ChaCha20Poly1305 context saving state is made of ChaCha20 state(first 16
 * bytes) and Poly1305(next 32 bytes).
 */
#define CHACHAPOLY_CTX_STATE_SZ (16 + 32)

#define CHACHA20_CTX_STATE_SZ (16)

/** Mode Register value for context loading */
#define CHACHAPOLY_MODEID_CTX_LOAD (BA417_MODEID_CTX_LOAD)
/** Mode Register value for context saving */
#define CHACHAPOLY_MODEID_CTX_SAVE (BA417_MODEID_CTX_SAVE)

#define BA417_MODE_CHACHA20POLY1305 (0)
#define BA417_MODE_CHACHA20	    (1)

static const char zeros[SX_CHACHAPOLY_TAG_SIZE] = {0};

static int lenAlenC_chachapoly(size_t aadsz, size_t datasz, uint8_t *out);

static const struct sx_aead_cmdma_tags ba417aeadtags = {
	.cfg = DMATAG_BA417 | DMATAG_CONFIG(0x00),
	.key = DMATAG_BA417 | DMATAG_CONFIG(0x04),
	.iv_or_state = DMATAG_BA417 | DMATAG_CONFIG(0x28),
	.nonce = DMATAG_BA417 | DMATAG_CONFIG(0x2C),
	.aad = DMATAG_BA417 | DMATAG_DATATYPE_HEADER,
	.tag = DMATAG_BA417,
	.data = DMATAG_BA417,
};

static const struct sx_aead_cmdma_cfg ba417chachapolycfg = {
	.decr = CM_CFG_DECRYPT << 2,
	.mode = BA417_MODE_CHACHA20POLY1305,
	.dmatags = &ba417aeadtags,
	.verifier = zeros,
	.lenAlenC = lenAlenC_chachapoly,
	.ctxsave = CHACHAPOLY_MODEID_CTX_SAVE,
	.ctxload = CHACHAPOLY_MODEID_CTX_LOAD,
	.granularity = SX_CHACHAPOLY_CTX_GRANULARITY_SZ,
	.statesz = CHACHAPOLY_CTX_STATE_SZ,
	.inputminsz = 0,
};

static const struct sx_blkcipher_cmdma_tags ba417blkciphertags = {
	.cfg = DMATAG_BA417 | DMATAG_CONFIG(0x00),
	.key = DMATAG_BA417 | DMATAG_CONFIG(0x04),
	.iv_or_state = DMATAG_BA417 | DMATAG_CONFIG(0x28),
	.data = DMATAG_BA417,
};

static const struct sx_blkcipher_cmdma_cfg ba417chacha20cfg = {
	.decr = CM_CFG_DECRYPT << 2,
	.ctxsave = CHACHAPOLY_MODEID_CTX_SAVE,
	.ctxload = CHACHAPOLY_MODEID_CTX_LOAD,
	.dmatags = &ba417blkciphertags,
	.statesz = CHACHA20_CTX_STATE_SZ,
	.mode = 0x01,
	.inminsz = 1,
	.granularity = 1,
	.blocksz = SX_CHACHAPOLY_BLOCK_SZ,
};

static int lenAlenC_chachapoly(size_t aadsz, size_t datasz, uint8_t *out)
{
	uint32_t i = 0;

	for (i = 0; i < 8; i++) {
		out[i] = aadsz & 0xFF;
		aadsz >>= 8;
	}
	out += 8;
	for (i = 0; i < 8; i++) {
		out[i] = datasz & 0xFF;
		datasz >>= 8;
	}

	return 1;
}

static int sx_aead_create_chacha20poly1305(struct sxaead *c, const struct sxkeyref *key,
					   const char *nonce, const uint32_t dir, size_t tagsz)
{

	if (key->sz != SX_CHACHAPOLY_KEY_SZ) {
		return SX_ERR_INVALID_KEY_SZ;
	}

	/* has countermeasures and the key need to be set before callling sx_aead_hw_reserve */
	c->has_countermeasures = false;
	c->key = key;
	sx_aead_hw_reserve(c);

	c->cfg = &ba417chachapolycfg;

	sx_cmdma_newcmd(&c->dma, c->descs, c->cfg->mode | dir, c->cfg->dmatags->cfg);
	ADD_CFGDESC(c->dma, key->key, SX_CHACHAPOLY_KEY_SZ, c->cfg->dmatags->key);

	/* In AEAD context, for BA417, the counter that must be provided and
	 * initialized with 1. counter size is 4 bytes. Starting at position 16
	 * due to lenAlenC that uses first 16 bytes of extramem
	 */
	c->extramem[16] = 0;
	c->extramem[17] = 0;
	c->extramem[18] = 0;
	c->extramem[19] = 1;

	ADD_INDESC_PRIV(c->dma, OFFSET_EXTRAMEM(c) + 16, SX_CHACHAPOLY_COUNTER_SIZE,
			c->cfg->dmatags->iv_or_state);
	ADD_CFGDESC(c->dma, nonce, SX_CHACHAPOLY_NONCE_SZ, c->cfg->dmatags->nonce);

	c->tagsz = tagsz;
	c->expectedtag = c->cfg->verifier;
	c->discardaadsz = 0;
	c->totalaadsz = 0;
	c->datainsz = 0;
	c->dataintotalsz = 0;

	return SX_OK;
}

static int sx_blkcipher_create_chacha20(struct sxblkcipher *c, struct sxkeyref *key,
					const char *counter, const char *nonce, const uint32_t dir)
{
	if (key->sz != SX_CHACHAPOLY_KEY_SZ) {
		return SX_ERR_INVALID_KEY_SZ;
	}

	memcpy(&c->key, key, sizeof(c->key));
	sx_hw_reserve(&c->dma);
	c->cfg = &ba417chacha20cfg;

	sx_cmdma_newcmd(&c->dma, c->descs, BA417_MODE_CHACHA20 | dir, c->cfg->dmatags->cfg);

	ADD_CFGDESC(c->dma, key->key, SX_CHACHAPOLY_KEY_SZ, c->cfg->dmatags->key);
	ADD_CFGDESC(c->dma, counter, SX_CHACHAPOLY_COUNTER_SIZE, c->cfg->dmatags->iv_or_state);
	ADD_CFGDESC(c->dma, nonce, SX_CHACHAPOLY_NONCE_SZ, DMATAG_BA417 | DMATAG_CONFIG(0x2C));

	c->textsz = 0;

	return SX_OK;
}

int sx_aead_create_chacha20poly1305_enc(struct sxaead *c, const struct sxkeyref *key,
					const char *nonce, size_t tagsz)
{
	return sx_aead_create_chacha20poly1305(c, key, nonce, 0, tagsz);
}

int sx_aead_create_chacha20poly1305_dec(struct sxaead *c, const struct sxkeyref *key,
					const char *nonce, size_t tagsz)
{
	return sx_aead_create_chacha20poly1305(c, key, nonce, ba417chachapolycfg.decr, tagsz);
}

int sx_blkcipher_create_chacha20_enc(struct sxblkcipher *c, struct sxkeyref *key,
				     const char *counter, const char *nonce)
{
	return sx_blkcipher_create_chacha20(c, key, counter, nonce, CM_CFG_ENCRYPT);
}

int sx_blkcipher_create_chacha20_dec(struct sxblkcipher *c, struct sxkeyref *key,
				     const char *counter, const char *nonce)
{
	return sx_blkcipher_create_chacha20(c, key, counter, nonce, ba417chacha20cfg.decr);
}
