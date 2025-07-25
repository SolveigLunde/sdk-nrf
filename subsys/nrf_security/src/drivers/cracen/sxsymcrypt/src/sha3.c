/*
 *  Copyright (c) 2024 Nordic Semiconductor ASA
 *
 *  SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <sxsymcrypt/hash.h>
#include <sxsymcrypt/hashdefs.h>
#include <cracen/statuscodes.h>
#include "crypmasterregs.h"
#include "hw.h"
#include "cmdma.h"

static const struct sx_digesttags ba418tags = {.cfg = DMATAG_BA418 | DMATAG_CONFIG(0),
					       .initialstate = DMATAG_BA418 | DMATAG_DATATYPE(1) |
							       DMATAG_LAST,
					       .data = DMATAG_BA418 | DMATAG_DATATYPE(0)};

/**
 * SHA3/SHAKE padding according to standard sha3 padding scheme described
 * in https://nvlpubs.nist.gov/nistpubs/FIPS/NIST.FIPS.202.pdf
 * (section 5.1 and B.2).
 */
static size_t fips202_pad(unsigned char prefix, unsigned char suffix, size_t capacity, size_t msgsz,
			  unsigned char *padding)
{
	size_t r = (1600 / 8) - capacity;
	size_t q;
	size_t i = 0;

	/* The number of bytes to be appended, denoted q, is in {1, 2, ..., r}.
	 */
	q = r - (msgsz % r);

	*padding = prefix;
	for (i = 1; i < q; i++) {
		padding++;
		*padding = 0;
	}
	*padding |= suffix;
	return q;
}

/** Byte to be added at the beginning of the padding in SHA3 mode */
#define SHA3_MODE_PREFIX 0x06

/** Byte to be added at the end of the padding in SHA3 mode */
#define SHA3_MODE_SUFFIX 0x80

/** Byte to be added at the beginning of the padding for SHAKE. */
#define SHAKE_MODE_PREFIX 0x1F

/** Byte to be added at the end of the padding for SHAKE. */
#define SHAKE_MODE_SUFFIX 0x80

#define SHA3_SAVE_CONTEXT	   (1 << 6)
#define SHA3_SHAKE_ENABLE	   (1 << 4)
#define SHA3_MODE(x)		   ((x) << 0)
#define SHA3_MODE_SHAKE(x, outlen) ((x) | SHA3_SHAKE_ENABLE | ((outlen) << 8))
#define SHA3_SW_PAD		   0

static void shake256_digest(struct sxhash *c, char *digest)
{
	unsigned char *padding = (unsigned char *)&c->extramem;
	int padsz;

	/* For SHAKE256, the capacity is 64 bytes. */
	padsz = fips202_pad(SHAKE_MODE_PREFIX, SHAKE_MODE_SUFFIX, 64, c->feedsz, padding);

	/* Use ADD_INDESC_PRIV_RAW instead of ADD_INDESC_PRIV.
	 * BA418 hardware cannot work with ADD_INDESC_PRIV as BA418 does not
	 * support byte ignore flags.
	 */
	ADD_INDESC_PRIV_RAW(c->dma, OFFSET_EXTRAMEM(c), padsz, c->dmatags->data);

	ADD_OUTDESCA(c->dma, digest, c->algo->digestsz, CMDMA_BA413_BUS_MSK);
}

static void sha3_digest(struct sxhash *c, char *digest)
{
	unsigned char *padding = (unsigned char *)&c->extramem;
	int padsz;

	padsz = fips202_pad(SHA3_MODE_PREFIX, SHA3_MODE_SUFFIX, 2 * c->algo->digestsz, c->feedsz,
			    padding);
	/* Use ADD_INDESC_PRIV_RAW instead of ADD_INDESC_PRIV.
	 * BA418 hardware cannot work with ADD_INDESC_PRIV as BA418 does not
	 * support byte ignore flags.
	 */
	ADD_INDESC_PRIV_RAW(c->dma, OFFSET_EXTRAMEM(c), padsz, c->dmatags->data);

	ADD_OUTDESCA(c->dma, digest, c->algo->digestsz, CMDMA_BA413_BUS_MSK);
}

static int sx_hash_create_ba418(struct sxhash *c, size_t csz)
{
	if ((csz < sizeof(*c)) || (c->algo->maxpadsz > sizeof(c->extramem))) {
		return SX_ERR_ALLOCATION_TOO_SMALL;
	}

	sx_hw_reserve(&c->dma);

	c->dmatags = &ba418tags;
	sx_cmdma_newcmd(&c->dma, c->descs, c->algo->cfgword, c->dmatags->cfg);
	c->digest = sha3_digest;
	c->feedsz = 0;
	c->totalfeedsz = 0;

	c->cntindescs = 2; /* reserve 1 extra descriptor for padding */

	return SX_OK;
}

static int sx_hash_create_ba418_shake256(struct sxhash *c, size_t csz)
{
	int r = sx_hash_create_ba418(c, csz);

	if (r != SX_OK) {
		return r;
	}

	c->digest = shake256_digest;

	return r;
}

const struct sxhashalg sxhashalg_sha3_224 = {
	SHA3_MODE(6), SHA3_SW_PAD, SHA3_SAVE_CONTEXT, SX_HASH_DIGESTSZ_SHA3_224,
	SX_HASH_BLOCKSZ_SHA3_224, 200, 144, sx_hash_create_ba418
};

const struct sxhashalg sxhashalg_sha3_256 = {
	SHA3_MODE(7), SHA3_SW_PAD, SHA3_SAVE_CONTEXT, SX_HASH_DIGESTSZ_SHA3_256,
	SX_HASH_BLOCKSZ_SHA3_256, 200, 136, sx_hash_create_ba418
};

const struct sxhashalg sxhashalg_sha3_384 = {
	SHA3_MODE(11), SHA3_SW_PAD, SHA3_SAVE_CONTEXT, SX_HASH_DIGESTSZ_SHA3_384,
	SX_HASH_BLOCKSZ_SHA3_384, 200, 104, sx_hash_create_ba418
};

const struct sxhashalg sxhashalg_sha3_512 = {
	SHA3_MODE(15), SHA3_SW_PAD, SHA3_SAVE_CONTEXT, SX_HASH_DIGESTSZ_SHA3_512,
	SX_HASH_BLOCKSZ_SHA3_512, 200, 72, sx_hash_create_ba418
};

const struct sxhashalg sxhashalg_shake256_114 = {
	SHA3_MODE_SHAKE(7, 114), SHA3_SW_PAD, SHA3_SAVE_CONTEXT, 114,
	SX_HASH_BLOCKSZ_SHA3_256, 200, 136, sx_hash_create_ba418_shake256
};
