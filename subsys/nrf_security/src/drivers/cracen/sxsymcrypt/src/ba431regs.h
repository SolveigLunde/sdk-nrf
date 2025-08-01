/** BA431 TRNG registers defines.
 *
 * @file
 * @copyright Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BA431REGS_H
#define BA431REGS_H

/** Registers configuration */

/** Control: Control register */
#define BA431_REG_Control_OFST			(0x00u)
/** FIFOLevel: FIFO level register. */
#define BA431_REG_FIFOLevel_OFST		(0x4u)
/** FIFOThreshold: FIFO threshold register. */
#define BA431_REG_FIFOThreshold_OFST		(0x8u)
/** FIFODepth: FIFO depth register. */
#define BA431_REG_FIFODepth_OFST		(0xCu)
/** Key0: Key register (MSBu). */
#define BA431_REG_Key0_OFST			(0x10u)
/** Key1: Key register. */
#define BA431_REG_Key1_OFST			(0x14u)
/** Key2: Key register. */
#define BA431_REG_Key2_OFST			(0x18u)
/** Key3: Key register (LSBu). */
#define BA431_REG_Key3_OFST			(0x1Cu)
/** TestData: Test data register. */
#define  BA431_REG_TestData_OFST                (0x20u)
/** Status: Status register. */
#define BA431_REG_Status_OFST			(0x30u)
/** InitWaitVal: Initial wait counter value. */
#define BA431_REG_InitWaitVal_OFST		(0x34u)
/** SwOffTmrVal: Switch off timer value. */
#define BA431_REG_SwOffTmrVal_OFST		(0x40u)
/** ClkDiv: Sample clock divider. */
#define BA431_REG_ClkDiv_OFST			(0x44u)
/** HwConfig: Hardware configuration register. */
#define  BA431_REG_HwConfig_OFST                (0x58u)
/** HwVersion: Hardware release register. */
#define  BA431_REG_HwVersion_OFST               (0x7Cu)
/** FIFO Data */
#define BA431_REG_FIFODATA_OFST			(0x80u)
/** Enable: Enable the NDRNG. */
#define BA431_FLD_Control_Enable_MASK		(0x1u)
/** Conditioning: Conditioning function bypass. */
#define  BA431_FLD_Control_Conditioning_Bypass_MASK (0x8u)
/** SoftRst: Software reset. */
#define BA431_FLD_Control_SoftRst_MASK		(0x100u)
/** Blending: Select blending method. */
#define  BA431_FLD_Control_Blending_MASK        (0xC000u)
#define  BA431_FLD_Control_Blending_LSB         (0x1Eu)
/** Nb128BitBlocks: Number of 128 bit blocks used in AES-CBCMAC
 * post-processing. This value cannot be zero.
 */
#define BA431_FLD_Control_Nb128BitBlocks_LSB	(16u)
#define BA431_FLD_Control_Nb128BitBlocks_MASK	(0xF0000u)
/** State: State of the control FSM. */
#define BA431_FLD_Status_State_LSB		(1u)
#define BA431_FLD_Status_State_MASK		(0xEu)
/** TestEn: Test enabled mask, enabled when set to 1. */
#define  BA431_FLD_Control_TestEn_MASK          (0x4u)
/** TestDataBusy: test data status bit, busy when set to 1. */
#define  BA431_FLD_Status_TestDataBusy_MASK     (0x1u)
/** ForceActiveROs: Force oscillators to run when FIFO is full. */
#define BA431_FLD_Control_ForceActiveROs_MASK	(0x800u)
/** HealthTestBypass: Bypass NIST tests such that the results of the start-up
 * and online test do not affect the FSM state.
 */
#define BA431_FLD_Control_HealthTestBypass_MASK (0x1000u)
/** AIS31Bypass: Bypass AIS31 tests such that the results of the start-up
 * and online tests do not affect the FSM state.
 */
#define BA431_FLD_Control_AIS31Bypass_MASK	(0x2000u)
/** Log2NbOfShares: log2 of number of shares. */
#define  BA431_REG_HwConfig_Log2NumShares_LSB   (16u)
#define  BA431_REG_HwConfig_Log2NumShares_MASK  (0x3u)

/** BA431 operating states */
#define BA431_STATE_RESET	   (0x00u)
#define BA431_STATE_STARTUP	   (0x01u)
#define BA431_STATE_IDLE_RINGS_ON  (0x02u)
#define BA431_STATE_IDLE_RINGS_OFF (0x03u)
#define BA431_STATE_FILL_FIFO	   (0x04u)
#define BA431_STATE_ERROR	   (0x05u)

#endif
