/*
 * Copyright (c) 2025 Nordic Semiconductor ASA.
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

SECTION_PROLOGUE(.align16,,)
{
	. = (ALIGN(16) > 0 ? ALIGN(16) : 16) - 1;
	BYTE(0);
} GROUP_LINK_IN(ROMABLE_REGION)
