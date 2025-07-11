/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <suit_execution_mode.h>

static suit_execution_mode_t current_execution_mode = EXECUTION_MODE_STARTUP;

suit_execution_mode_t suit_execution_mode_get(void)
{
	return current_execution_mode;
}

suit_plat_err_t suit_execution_mode_set(suit_execution_mode_t mode)
{
	if ((mode > EXECUTION_MODE_STARTUP) && (mode <= EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP)) {
		current_execution_mode = mode;

		return SUIT_PLAT_SUCCESS;
	}

	return SUIT_PLAT_ERR_INVAL;
}

void suit_execution_mode_startup_failed(void)
{
	switch (current_execution_mode) {
	/* SUIT boot or update not yet started. */
	case EXECUTION_MODE_STARTUP:
	/* System is unprovisioned, SUIT updates Nordic components. */
	case EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP:
	/* SUIT processes update candiadate. */
	case EXECUTION_MODE_INSTALL:
	/* SUIT processes recovery as foreground update. */
	case EXECUTION_MODE_INSTALL_FOREGROUND_DFU:
	/* SUIT processes recovery update. */
	case EXECUTION_MODE_INSTALL_RECOVERY:
	/* SUIT boots from root manifest. */
	case EXECUTION_MODE_INVOKE:
	/* SUIT boots from recovery manifest as foreground update. */
	case EXECUTION_MODE_INVOKE_FOREGROUND_DFU:
	/* SUIT boots from recovery manifest. */
	case EXECUTION_MODE_INVOKE_RECOVERY:
		current_execution_mode = EXECUTION_MODE_FAIL_STARTUP;
		break;

	/* System not booted, application MPI missing. */
	case EXECUTION_MODE_FAIL_NO_MPI:
	/* System not booted, invalid MPI format. */
	case EXECUTION_MODE_FAIL_MPI_INVALID:
	/* System not booted, Nordic and root class IDs not configured. */
	case EXECUTION_MODE_FAIL_MPI_INVALID_MISSING:
	/* System not booted, MPI misconfigured. */
	case EXECUTION_MODE_FAIL_MPI_UNSUPPORTED:
	/* System not booted, unable to boot recovery manifest as foreground update. */
	case EXECUTION_MODE_FAIL_INVOKE_FOREGROUND_DFU:
	/* System not booted, unable to boot recovery manifest. */
	case EXECUTION_MODE_FAIL_INVOKE_RECOVERY:
	/* System booted from root manifest. */
	case EXECUTION_MODE_POST_INVOKE:
	/* System booted from recovery manifest as foreground update. */
	case EXECUTION_MODE_POST_INVOKE_FOREGROUND_DFU:
	/* System booted from recovery manifest. */
	case EXECUTION_MODE_POST_INVOKE_RECOVERY:
	/* System failed before invoking SUIT orchestrator. */
	case EXECUTION_MODE_FAIL_STARTUP:
		break;
		/* default case deliberately excluded to get warnings if missing an enum case. */
	}
}

bool suit_execution_mode_booting(void)
{
	switch (current_execution_mode) {
	/* SUIT processes update candiadate. */
	case EXECUTION_MODE_INSTALL:
	/* SUIT processes recovery as foreground update. */
	case EXECUTION_MODE_INSTALL_FOREGROUND_DFU:
	/* SUIT processes recovery update. */
	case EXECUTION_MODE_INSTALL_RECOVERY:
	/* System is unprovisioned, SUIT updates Nordic components. */
	case EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP:
	/* System booted from root manifest. */
	case EXECUTION_MODE_POST_INVOKE:
	/* System booted from recovery manifest as foreground update. */
	case EXECUTION_MODE_POST_INVOKE_FOREGROUND_DFU:
	/* System booted from recovery manifest. */
	case EXECUTION_MODE_POST_INVOKE_RECOVERY:
	/* System not booted, application MPI missing. */
	case EXECUTION_MODE_FAIL_NO_MPI:
	/* System not booted, invalid MPI format. */
	case EXECUTION_MODE_FAIL_MPI_INVALID:
	/* System not booted, Nordic and root class IDs not configured. */
	case EXECUTION_MODE_FAIL_MPI_INVALID_MISSING:
	/* System not booted, MPI misconfigured. */
	case EXECUTION_MODE_FAIL_MPI_UNSUPPORTED:
	/* System not booted, unable to boot recovery manifest as foreground update. */
	case EXECUTION_MODE_FAIL_INVOKE_FOREGROUND_DFU:
	/* System not booted, unable to boot recovery manifest. */
	case EXECUTION_MODE_FAIL_INVOKE_RECOVERY:
	/* System failed before invoking SUIT orchestrator. */
	case EXECUTION_MODE_FAIL_STARTUP:
		return false;

	/* SUIT boot or update not yet started. */
	case EXECUTION_MODE_STARTUP:
	/* SUIT boots from root manifest. */
	case EXECUTION_MODE_INVOKE:
	/* SUIT boots from recovery manifest as foreground update. */
	case EXECUTION_MODE_INVOKE_FOREGROUND_DFU:
	/* SUIT boots from recovery manifest. */
	case EXECUTION_MODE_INVOKE_RECOVERY:
		break;
		/* default case deliberately excluded to get warnings if missing an enum case. */
	}

	return true;
}

bool suit_execution_mode_updating(void)
{
	switch (current_execution_mode) {
	/* SUIT boots from root manifest. */
	case EXECUTION_MODE_INVOKE:
	/* SUIT boots from recovery manifest as foreground update. */
	case EXECUTION_MODE_INVOKE_FOREGROUND_DFU:
	/* SUIT boots from recovery manifest. */
	case EXECUTION_MODE_INVOKE_RECOVERY:
	/* System booted from root manifest. */
	case EXECUTION_MODE_POST_INVOKE:
	/* System booted from recovery manifest as foreground update. */
	case EXECUTION_MODE_POST_INVOKE_FOREGROUND_DFU:
	/* System booted from recovery manifest. */
	case EXECUTION_MODE_POST_INVOKE_RECOVERY:
	/* System not booted, application MPI missing. */
	case EXECUTION_MODE_FAIL_NO_MPI:
	/* System not booted, invalid MPI format. */
	case EXECUTION_MODE_FAIL_MPI_INVALID:
	/* System not booted, Nordic and root class IDs not configured. */
	case EXECUTION_MODE_FAIL_MPI_INVALID_MISSING:
	/* System not booted, MPI misconfigured. */
	case EXECUTION_MODE_FAIL_MPI_UNSUPPORTED:
	/* System not booted, unable to boot recovery manifest as foreground update. */
	case EXECUTION_MODE_FAIL_INVOKE_FOREGROUND_DFU:
	/* System not booted, unable to boot recovery manifest. */
	case EXECUTION_MODE_FAIL_INVOKE_RECOVERY:
	/* System failed before invoking SUIT orchestrator. */
	case EXECUTION_MODE_FAIL_STARTUP:
		return false;

	/* SUIT boot or update not yet started. */
	case EXECUTION_MODE_STARTUP:
	/* SUIT processes update candiadate. */
	case EXECUTION_MODE_INSTALL:
	/* SUIT processes recovery as foreground update. */
	case EXECUTION_MODE_INSTALL_FOREGROUND_DFU:
	/* SUIT processes recovery update. */
	case EXECUTION_MODE_INSTALL_RECOVERY:
	/* System is unprovisioned, SUIT updates Nordic components. */
	case EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP:
		break;
		/* default case deliberately excluded to get warnings if missing an enum case. */
	}

	return true;
}

bool suit_execution_mode_failed(void)
{
	switch (current_execution_mode) {
	/* SUIT boot or update not yet started. */
	case EXECUTION_MODE_STARTUP:
	/* SUIT processes update candiadate. */
	case EXECUTION_MODE_INSTALL:
	/* SUIT processes recovery as foreground update. */
	case EXECUTION_MODE_INSTALL_FOREGROUND_DFU:
	/* SUIT processes recovery update. */
	case EXECUTION_MODE_INSTALL_RECOVERY:
	/* SUIT boots from root manifest. */
	case EXECUTION_MODE_INVOKE:
	/* SUIT boots from recovery manifest as foreground update. */
	case EXECUTION_MODE_INVOKE_FOREGROUND_DFU:
	/* SUIT boots from recovery manifest. */
	case EXECUTION_MODE_INVOKE_RECOVERY:
	/* System booted from root manifest. */
	case EXECUTION_MODE_POST_INVOKE:
	/* System booted from recovery manifest as foreground update. */
	case EXECUTION_MODE_POST_INVOKE_FOREGROUND_DFU:
	/* System booted from recovery manifest. */
	case EXECUTION_MODE_POST_INVOKE_RECOVERY:
	/* System is unprovisioned, SUIT updates Nordic components. */
	case EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP:
		return false;

	/* System not booted, application MPI missing. */
	case EXECUTION_MODE_FAIL_NO_MPI:
	/* System not booted, invalid MPI format. */
	case EXECUTION_MODE_FAIL_MPI_INVALID:
	/* System not booted, Nordic and root class IDs not configured. */
	case EXECUTION_MODE_FAIL_MPI_INVALID_MISSING:
	/* System not booted, MPI misconfigured. */
	case EXECUTION_MODE_FAIL_MPI_UNSUPPORTED:
	/* System not booted, unable to boot recovery manifest as foreground update. */
	case EXECUTION_MODE_FAIL_INVOKE_FOREGROUND_DFU:
	/* System not booted, unable to boot recovery manifest. */
	case EXECUTION_MODE_FAIL_INVOKE_RECOVERY:
	/* System failed before invoking SUIT orchestrator. */
	case EXECUTION_MODE_FAIL_STARTUP:
		break;
		/* default case deliberately excluded to get warnings if missing an enum case. */
	}

	return true;
}

suit_boot_mode_t suit_execution_mode_to_boot_mode(suit_execution_mode_t exec_mode)
{
	switch (exec_mode) {
	case EXECUTION_MODE_INVOKE:
		return SUIT_BOOT_MODE_INVOKE;
	case EXECUTION_MODE_INVOKE_FOREGROUND_DFU:
		return SUIT_BOOT_MODE_INVOKE_FOREGROUND_DFU;
	case EXECUTION_MODE_INVOKE_RECOVERY:
		return SUIT_BOOT_MODE_INVOKE_RECOVERY;
	case EXECUTION_MODE_INSTALL:
		return SUIT_BOOT_MODE_INSTALL;
	case EXECUTION_MODE_INSTALL_FOREGROUND_DFU:
		return SUIT_BOOT_MODE_INSTALL_FOREGROUND_DFU;
	case EXECUTION_MODE_INSTALL_RECOVERY:
		return SUIT_BOOT_MODE_INSTALL_RECOVERY;
	case EXECUTION_MODE_POST_INVOKE:
		return SUIT_BOOT_MODE_POST_INVOKE;
	case EXECUTION_MODE_POST_INVOKE_FOREGROUND_DFU:
		return SUIT_BOOT_MODE_POST_INVOKE_FOREGROUND_DFU;
	case EXECUTION_MODE_POST_INVOKE_RECOVERY:
		return SUIT_BOOT_MODE_POST_INVOKE_RECOVERY;
	case EXECUTION_MODE_FAIL_NO_MPI:
		return SUIT_BOOT_MODE_FAIL_NO_MPI;
	case EXECUTION_MODE_FAIL_MPI_INVALID:
		return SUIT_BOOT_MODE_FAIL_MPI_INVALID;
	case EXECUTION_MODE_FAIL_MPI_INVALID_MISSING:
		return SUIT_BOOT_MODE_FAIL_MPI_INVALID_MISSING;
	case EXECUTION_MODE_FAIL_MPI_UNSUPPORTED:
		return SUIT_BOOT_MODE_FAIL_MPI_UNSUPPORTED;
	case EXECUTION_MODE_FAIL_INVOKE_FOREGROUND_DFU:
		return SUIT_BOOT_MODE_FAIL_INVOKE_FOREGROUND_DFU;
	case EXECUTION_MODE_FAIL_INVOKE_RECOVERY:
		return SUIT_BOOT_MODE_FAIL_INVOKE_RECOVERY;
	case EXECUTION_MODE_FAIL_INSTALL_NORDIC_TOP:
		return SUIT_BOOT_MODE_FAIL_INSTALL_NORDIC_TOP;
	case EXECUTION_MODE_FAIL_STARTUP:
		return SUIT_BOOT_MODE_FAIL_STARTUP;
	default:
		break;
	}

	return SUIT_BOOT_MODE_UNKNOWN;
}
