/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/ztest.h>

#include <suit_orchestrator.h>
#include <suit_storage.h>
#include <suit_execution_mode.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <suit_plat_mem_util.h>
#include "test_common.h"

#if DT_NODE_EXISTS(DT_NODELABEL(cpuapp_suit_storage))
#define SUIT_STORAGE_OFFSET FIXED_PARTITION_OFFSET(cpuapp_suit_storage)
#define SUIT_STORAGE_SIZE   FIXED_PARTITION_SIZE(cpuapp_suit_storage)
#else
#define SUIT_STORAGE_OFFSET FIXED_PARTITION_OFFSET(suit_storage)
#define SUIT_STORAGE_SIZE   FIXED_PARTITION_SIZE(suit_storage)
#endif
#define SUIT_STORAGE_ADDRESS suit_plat_mem_nvm_ptr_get(SUIT_STORAGE_OFFSET)

/* Valid envelope */
extern uint8_t manifest_valid_buf[];
extern size_t manifest_valid_len;

static void setup_erased_flash(void)
{
	/* Execute SUIT storage init, so the MPI area is copied into a backup region. */
	int err = suit_storage_init();

	zassert_equal(SUIT_PLAT_SUCCESS, err, "Failed to init and backup suit storage (%d)", err);

	/* Clear the whole application area */
	const struct device *fdev = SUIT_PLAT_INTERNAL_NVM_DEV;

	zassert_not_null(fdev, "Unable to find a driver to erase storage area");

	err = flash_erase(fdev, SUIT_STORAGE_OFFSET, SUIT_STORAGE_SIZE);
	zassert_equal(0, err, "Unable to erase storage before test execution");

	suit_plat_err_t ret = suit_storage_flags_clear(SUIT_FLAG_RECOVERY);

	zassert_equal(SUIT_PLAT_SUCCESS, ret,
		      "Unable to clear recovery flag before test execution");

	ret = suit_storage_flags_clear(SUIT_FLAG_FOREGROUND_DFU);
	zassert_equal(SUIT_PLAT_SUCCESS, ret,
		      "Unable to clear foreground DFU flag before test execution");

	/* Recover MPI area from the backup region. */
	err = suit_storage_init();
	zassert_equal(SUIT_PLAT_SUCCESS, err, "Failed to recover MPI area from backup (%d)", err);
}

static void setup_update_candidate(const uint8_t *buf, size_t len)
{
	zassert_not_null(buf, "NULL buf");

	suit_plat_mreg_t update_candidate[1] = {{
		.mem = buf,
		.size = len,
	}};

	suit_plat_err_t err =
		suit_storage_update_cand_set(update_candidate, ARRAY_SIZE(update_candidate));

	zassert_equal(SUIT_PLAT_SUCCESS, err,
		      "Unable to set update candidate before test execution (0x%x, %d)", buf, len);
}

static void setup_recovery_flag(void)
{
	zassert_equal(SUIT_PLAT_SUCCESS, suit_storage_flags_set(SUIT_FLAG_RECOVERY),
		      "Unable to set recovery flag before test execution");
}

static void setup_fdfu_flag(void)
{
	zassert_equal(SUIT_PLAT_SUCCESS, suit_storage_flags_set(SUIT_FLAG_FOREGROUND_DFU),
		      "Unable to set foreground DFU flag before test execution");
}

ZTEST_SUITE(orchestrator_init_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(orchestrator_init_tests, test_orchestrator_sdfw_empty_storage)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and update candidate flag is not set... */
	/* ... and emergency flag is not set */

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN regular boot is triggered... */
	zassert_equal(EXECUTION_MODE_INVOKE, suit_execution_mode_get(),
		      "Regular boot not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode indicates boot mode */
	zassert_equal(true, suit_execution_mode_booting(), "The device did not enter boot mode");
	/* ... and execution mode does not indicate update mode */
	zassert_equal(false, suit_execution_mode_updating(), "The device entered update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}

ZTEST(orchestrator_init_tests, test_orchestrator_sdfw_empty_storage_with_update_flag)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and valid update candidate in suit storage... */
	setup_update_candidate(manifest_valid_buf, manifest_valid_len);
	/* ... and emergency flag is not set */

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN regular update is triggered... */
	zassert_equal(EXECUTION_MODE_INSTALL, suit_execution_mode_get(),
		      "Regular update not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode does not indicate boot mode */
	zassert_equal(false, suit_execution_mode_booting(), "The device entered boot mode");
	/* ... and execution mode indicates update mode */
	zassert_equal(true, suit_execution_mode_updating(), "The device did not enter update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}

ZTEST(orchestrator_init_tests, test_orchestrator_sdfw_empty_storage_with_recovery_flag)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and update candidate flag is not set... */
	/* ... and emergency flag is set */
	setup_recovery_flag();

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN emergency recovery is triggered... */
	zassert_equal(EXECUTION_MODE_INVOKE_RECOVERY, suit_execution_mode_get(),
		      "Recovery mode not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode indicates boot mode */
	zassert_equal(true, suit_execution_mode_booting(), "The device did not enter boot mode");
	/* ... and execution mode does not indicate update mode */
	zassert_equal(false, suit_execution_mode_updating(), "The device entered update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}

ZTEST(orchestrator_init_tests, test_orchestrator_sdfw_empty_storage_with_update_recovery_flag)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and valid update candidate in suit storage... */
	setup_update_candidate(manifest_valid_buf, manifest_valid_len);
	/* ... and emergency flag is set */
	setup_recovery_flag();

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN emergency recovery update is triggered... */
	zassert_equal(EXECUTION_MODE_INSTALL_RECOVERY, suit_execution_mode_get(),
		      "Emergency recovery update not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode does not indicate boot mode */
	zassert_equal(false, suit_execution_mode_booting(), "The device entered boot mode");
	/* ... and execution mode indicates update mode */
	zassert_equal(true, suit_execution_mode_updating(), "The device did not enter update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}

ZTEST(orchestrator_init_tests, test_empty_storage_with_fdfu_flag)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and update candidate flag is not set... */
	/* ... and foreground DFU flag is set */
	setup_fdfu_flag();

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN foreground DFU is triggered... */
	zassert_equal(EXECUTION_MODE_INVOKE_FOREGROUND_DFU, suit_execution_mode_get(),
		      "Foreground DFU mode not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode indicates boot mode */
	zassert_equal(true, suit_execution_mode_booting(), "The device did not enter boot mode");
	/* ... and execution mode does not indicate update mode */
	zassert_equal(false, suit_execution_mode_updating(), "The device entered update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}

ZTEST(orchestrator_init_tests, test_empty_storage_with_update_fdfu_flag)
{
	/* GIVEN empty flash (suit storage is erased)... */
	setup_erased_flash();
	/* ... and valid update candidate in suit storage... */
	setup_update_candidate(manifest_valid_buf, manifest_valid_len);
	/* ... and foreground DFU flag is set */
	setup_fdfu_flag();

	/* WHEN orchestrator is initialized */
	int err = suit_orchestrator_init();

	/* THEN foreground DFU is triggered... */
	zassert_equal(EXECUTION_MODE_INSTALL_FOREGROUND_DFU, suit_execution_mode_get(),
		      "Foreground DFU update not triggered");
	/* ... and orchestrator is initialized */
	zassert_equal(0, err, "Orchestrator not initialized");
	/* ... and execution mode does not indicate a failed state */
	zassert_equal(false, suit_execution_mode_failed(), "The device entered failed mode");
	/* ... and execution mode does not indicate boot mode */
	zassert_equal(false, suit_execution_mode_booting(), "The device entered boot mode");
	/* ... and execution mode indicates update mode */
	zassert_equal(true, suit_execution_mode_updating(), "The device did not enter update mode");
	/* ... and the startup failure is correctly handled */
	check_startup_failure();
}
