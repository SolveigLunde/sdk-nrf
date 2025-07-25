/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ST25R3911B_REG_H_
#define ST25R3911B_REG_H_

/**
 * @file
 * @defgroup st25r3911b_nfc_reg ST25R3911B NFC Reader Registers and Commands
 * @{
 *
 * @brief Registers and commands for the ST25R3911B NFC Reader.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup st25r3911b_direct_command Direct command
 * @{
 *
 * @brief ST25R3911B direct command defined in IC documentation chapter 1.1.
 */
#define ST25R3911B_CMD_SET_DEFAULT 0xC1
#define ST25R3911B_CMD_CLEAR 0xC2
#define ST25R3911B_CMD_TX_WITH_CRC 0xC4
#define ST25R3911B_CMD_TX_WITHOUT_CRC 0xC5
#define ST25R3911B_CMD_TX_REQA 0xC6
#define ST25R3911B_CMD_TX_WUPA 0xC7
#define ST25R3911B_CMD_NFC_INITIAL_FIELD_ON 0xC8
#define ST25R3911B_CMD_NFC_RSP_FIELD_ON 0xC9
#define ST25R3911B_CMD_NFC_RSP_FILED_ON_0 0xCA
#define ST25R3911B_CMD_NORMAL_NFC_MODE 0xCB
#define ST25R3911B_CMD_ANALOG_PRESET 0xCC
#define ST25R3911B_CMD_MASK_RECEIVE_DATA 0xD0
#define ST25R3911B_CMD_UMNASK_RECEIVE_DATA 0xD1
#define ST25R3911B_CMD_MEASURE_AMPLITUDE 0xD3
#define ST25R3911B_CMD_SQUELCH 0xD4
#define ST25R3911B_CMD_RESET_RX_GAIN 0xD5
#define ST25R3911B_CMD_ADJUST_REGULATOR 0xD6
#define ST25R3911B_CMD_CALIBRATE_MODULATION 0xD7
#define ST25R3911B_CMD_CALIBRATE_ANTENNA 0xD8
#define ST25R3911B_CMD_MEASURE_PHASE 0xD9
#define ST25R3911B_CMD_CLEAR_RSSI 0xDA
#define ST25R3911B_CMD_TRANSPARENT_MODE 0xDC
#define ST25R3911B_CMD_CALIBRATE_CAP_SENSOR 0xDD
#define ST25R3911B_CMD_MEASURE_CAP 0xDE
#define ST25R3911B_CMD_MEASURE_POWER_SUPPLY 0xDF
#define ST25R3911B_CMD_START_GEN_PURPOSE_TIMER 0xE0
#define ST25R3911B_CMD_START_WAKE_UP_TIMER 0xE1
#define ST25R3911B_CMD_START_MASK_RX_TIMER 0xE2
#define ST25R3911B_CMD_START_NO_RSP_TIMER 0xE3
#define ST25R3911B_CMD_CLEAR_TEST_REG 0xFA
#define ST25R3911B_CMD_TEST_ACCESS 0xFC

/**
 * @}
 */

/** @defgroup st25r3911b_reg Registers
 * @{
 *
 * @brief The 6-bit register addresses, defined in IC documentation
 *        chapter 1.3.
 */
#define ST25R3911B_REG_IO_CONF1 0x00
#define ST25R3911B_REG_IO_CONF2 0x01
#define ST25R3911B_REG_OP_CTRL 0x02
#define ST25R3911B_REG_MODE_DEF 0x03
#define ST25R3911B_REG_BIT_RATE 0x04
#define ST25R3911B_REG_ISO14443A 0x05
#define ST25R3911B_REG_ISO14443B_1 0x06
#define ST25R3911B_REG_ISO14443B_2 0x07
#define ST25R3911B_REG_STREAM_MODE 0x08
#define ST25R3911B_REG_AUXILIARY 0x09
#define ST25R3911B_REG_RX_CONF1 0x0A
#define ST25R3911B_REG_RX_CONF2 0x0B
#define ST25R3911B_REG_RX_CONF3 0x0C
#define ST25R3911B_REG_RX_CONF4 0x0D
#define ST25R3911B_REG_MASK_RX_TIM 0x0E
#define ST25R3911B_REG_NO_RSP_TIM_REG1 0x0F
#define ST25R3911B_REG_NO_RSP_TIM_REG2 0x10
#define ST25R3911B_REG_TIM_CTRl 0x11
#define ST25R3911B_REG_GPT_1 0x12
#define ST25R3911B_REG_GPT_2 0x13
#define ST25R3911B_REG_MASK_MAIN_INT 0x14
#define ST25R3911B_REG_MASK_TIM_NFC_INT 0x15
#define ST25R3911B_REG_MASK_ERR_WAKE_UP_INT 0x16
#define ST25R3911B_REG_MAIN_INT 0x17
#define ST25R3911B_REG_TIM_NFC_INT 0x18
#define ST25R3911B_REG_ERR_WAKE_UP_INT 0x19
#define ST25R3911B_REG_FIFO_STATUS_1 0x1A
#define ST25R3911B_REG_FIFO_STATUS_2 0x1B
#define ST25R3911B_REG_COLLISION_DISP 0x1C
#define ST25R3911B_REG_NUM_TX_BYTES_REG1 0x1D
#define ST25R3911B_REG_NUM_TX_BYTES_REG2 0x1E
#define ST25R3911B_REG_NFCIP_BIT_RATE 0x1F
#define ST25R3911B_REG_AD_CONVERTER_OUT 0x20
#define ST25R3911B_REG_ANTENNA_CAL_CTRL 0x21
#define ST25R3911B_REG_ANTENNA_CAL_TARGET 0x22
#define ST25R3911B_REG_ANTENNA_CAL_DISP 0x23
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL 0x24
#define ST25R3911B_REG_AM_MOD_DEPTH_DISP 0x25
#define ST25R3911B_REG_RFO_AM_LVL_DEF 0x26
#define ST25R3911B_REG_RFO_NORMAL_LVL_DEF 0x27
#define ST25R3911B_REG_FIELD_THRESHOLD 0x29
#define ST25R3911B_REG_REGULATOR_CTRL 0x2A
#define ST25R3911B_REG_REGULATOR_TIM_DISP 0x2B
#define ST25R3911B_REG_RSSI_DISP 0x2C
#define ST25R3911B_REG_GAIN_REDUCTION_STATE 0x2D
#define ST25R3911B_REG_CAP_SENSOR_CTRL 0x2E
#define ST25R3911B_REG_CAP_SENSOR_DISP 0x2F
#define ST25R3911B_REG_AUXILIARY_DISP 0x30
#define ST25R3911B_REG_WAKE_UP_TIM 0x31
#define ST25R3911B_REG_AMP_MEASURE_CONF 0x32
#define ST25R3911B_REG_AMP_MEASURE_REF 0x33
#define ST25R3911B_REG_AMP_MEASURE_AUTO_AVE_DISP 0x34
#define ST25R3911B_REG_AMP_MEASURE_DISP 0x35
#define ST25R3911B_REG_PHASE_MEASURE_CONF 0x36
#define ST25R3911B_REG_PHASE_MEASURE_REF 0x37
#define ST25R3911B_REG_PHASE_MEASURE_AUTO_AVE_DISP 0x38
#define ST25R3911B_REG_PHASE_MEASURE_DISP 0x39
#define ST25R3911B_REG_CAP_MEASURE_CONF 0x3A
#define ST25R3911B_REG_CAP_MEASURE_REF 0x3B
#define ST25R3911B_REG_CAP_MEASURE_AUTO_AVE 0x3C
#define ST25R3911B_REG_CAP_MEASURE_DISPLAY 0x3D
#define ST25R3911B_REG_IC_IDENTITY 0x3F

/**
 * @}
 */

/** @defgroup st25r3911b_io_conf1_reg IO Configuration
 *            Register 1 bits
 * @{
 */
#define ST25R3911B_REG_IO_CONF1_LF_CLK_OFF BIT(0)
#define ST25R3911B_REG_IO_CONF1_OUT_CL0 BIT(1)
#define ST25R3911B_REG_IO_CONF1_OUT_CL1 BIT(2)
#define ST25R3911B_REG_IO_CONF1_OSC BIT(3)
#define ST25R3911B_REG_IO_CONF1_FIFO_LT BIT(4)
#define ST25R3911B_REG_IO_CONF1_FIFO_LR BIT(5)
#define ST25R3911B_REG_IO_CONF1_RFO_2 BIT(6)
#define ST25R3911B_REG_IO_CONF1_SINGLE BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_io_conf2_reg Configuration
 *            Register 2 bits
 * @{
 */
#define ST25R3911B_REG_IO_CONF2_SLOW_UP BIT(0)
#define ST25R3911B_REG_IO_CONF2_IO_18 BIT(2)
#define ST25R3911B_REG_IO_CONF2_MISO_PD1 BIT(3)
#define ST25R3911B_REG_IO_CONF2_MISO_PD2 BIT(4)
#define ST25R3911B_REG_IO_CONF2_VPSD_OFF BIT(6)
#define ST25R3911B_REG_IO_CONF2_SUP3V BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_op_ctrl_reg Operation Control
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_OP_CTRL_WU BIT(2)
#define ST25R3911B_REG_OP_CTRL_TX_EN BIT(3)
#define ST25R3911B_REG_OP_CTRL_RX_MAN BIT(4)
#define ST25R3911B_REG_OP_CTRL_RX_CHN BIT(5)
#define ST25R3911B_REG_OP_CTRL_RX_EN BIT(6)
#define ST25R3911B_REG_OP_CTRL_EN BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_mode_def_reg Mode Definition
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_MODE_DEF_NFC_AR BIT(0)
#define ST25R3911B_REG_MODE_DEF_OM0 BIT(3)
#define ST25R3911B_REG_MODE_DEF_OM1 BIT(4)
#define ST25R3911B_REG_MODE_DEF_OM2 BIT(5)
#define ST25R3911B_REG_MODE_DEF_OM3 BIT(6)
#define ST25R3911B_REG_MODE_DEF_TARG BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_bit_rate_reg Bit Rate Definition
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_BIT_RATE_RX_RATE0 BIT(0)
#define ST25R3911B_REG_BIT_RATE_RX_RATE1 BIT(1)
#define ST25R3911B_REG_BIT_RATE_RX_RATE2 BIT(2)
#define ST25R3911B_REG_BIT_RATE_RX_RATE3 BIT(3)
#define ST25R3911B_REG_BIT_RATE_TX_RATE0 BIT(4)
#define ST25R3911B_REG_BIT_RATE_TX_RATE1 BIT(5)
#define ST25R3911B_REG_BIT_RATE_TX_RATE2 BIT(6)
#define ST25R3911B_REG_BIT_RATE_TX_RATE3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_iso14443a_reg ISO14443A and NFC
 *            settings Register bits
 * @{
 */
#define ST25R3911B_REG_ISO14443A_ANTCL BIT(0)
#define ST25R3911B_REG_ISO14443A_PLEN0 BIT(1)
#define ST25R3911B_REG_ISO14443A_PLEN1 BIT(2)
#define ST25R3911B_REG_ISO14443A_PLEN2 BIT(3)
#define ST25R3911B_REG_ISO14443A_PLEN3 BIT(4)
#define ST25R3911B_REG_ISO14443A_NFC_F0 BIT(5)
#define ST25R3911B_REG_ISO14443A_NFC_F0 BIT(5)
#define ST25R3911B_REG_ISO14443A_NO_RX_PARITY BIT(6)
#define ST25R3911B_REG_ISO14443A_NO_TX_PARITY BIT(7)

/**
 * @}
 */

/**
 * @defgroup st25r3911b_iso14443b_1_reg ISO14443B settings
 *           Register 1 bits
 * @{
 */
#define	ST25R3911B_REG_ISO14443B_1_RX_ST_OM BIT(0)
#define	ST25R3911B_REG_ISO14443B_1_HALF BIT(1)
#define	ST25R3911B_REG_ISO14443B_1_EOF BIT(2)
#define	ST25R3911B_REG_ISO14443B_1_SOF0 BIT(3)
#define	ST25R3911B_REG_ISO14443B_1_SOF1 BIT(4)
#define	ST25R3911B_REG_ISO14443B_1_EGT0 BIT(5)
#define	ST25R3911B_REG_ISO14443B_1_EGT1 BIT(6)
#define	ST25R3911B_REG_ISO14443B_1_EGT2 BIT(7)
/**
 * @}
 */

/** @defgroup st25r3911b_iso14443b_2_reg ISO14443B and FeliCa
 *            settings Register 2 bits
 * @{
 */
#define	ST25R3911B_REG_ISO14443B_2_F_P0 BIT(0)
#define	ST25R3911B_REG_ISO14443B_2_F_P1 BIT(1)
#define	ST25R3911B_REG_ISO14443B_2_PHC_TH BIT(2)
#define	ST25R3911B_REG_ISO14443B_2_EOF12 BIT(3)
#define	ST25R3911B_REG_ISO14443B_2_NO_EOF BIT(4)
#define	ST25R3911B_REG_ISO14443B_2_NO_SOF BIT(5)
#define	ST25R3911B_REG_ISO14443B_2_TR1_0 BIT(6)
#define	ST25R3911B_REG_ISO14443B_2_TR2_1 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_stream_mode_reg Stream Mode Definition
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_STREAM_MODE_STX0 BIT(0)
#define ST25R3911B_REG_STREAM_MODE_STX1 BIT(1)
#define ST25R3911B_REG_STREAM_MODE_STX2 BIT(2)
#define ST25R3911B_REG_STREAM_MODE_STX3 BIT(3)
#define ST25R3911B_REG_STREAM_MODE_SCP0 BIT(4)
#define ST25R3911B_REG_STREAM_MODE_SCP1 BIT(5)
#define ST25R3911B_REG_STREAM_MODE_SCF0 BIT(6)
#define ST25R3911B_REG_STREAM_MODE_SCF1 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_auxiliary_reg Auxiliary Register bits
 * @{
 */
#define ST25R3911B_REG_AUXILIARY_NFC_N0 BIT(0)
#define ST25R3911B_REG_AUXILIARY_NFC_N1 BIT(1)
#define ST25R3911B_REG_AUXILIARY_RX_TOL BIT(2)
#define ST25R3911B_REG_AUXILIARY_OOK_HR BIT(3)
#define ST25R3911B_REG_AUXILIARY_EN_FD BIT(4)
#define ST25R3911B_REG_AUXILIARY_TR_AM BIT(5)
#define ST25R3911B_REG_AUXILIARY_CRC_2_FIFO BIT(6)
#define ST25R3911B_REG_AUXILIARY_NO_CRC_RX BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_rx_conf1_reg Receiver Configuration
 *            Register 1 bits
 * @{
 */
#define ST25R3911B_REG_RX_CONF1_Z12K BIT(0)
#define ST25R3911B_REG_RX_CONF1_H80 BIT(1)
#define ST25R3911B_REG_RX_CONF1_H200 BIT(2)
#define ST25R3911B_REG_RX_CONF1_LP0 BIT(3)
#define ST25R3911B_REG_RX_CONF1_LP1 BIT(4)
#define ST25R3911B_REG_RX_CONF1_LP2 BIT(5)
#define ST25R3911B_REG_RX_CONF1_AMD_SEL BIT(6)
#define ST25R3911B_REG_RX_CONF1_CH_SEL BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_rx_conf2_reg Receiver Configuration
 *            Register 2 bits
 * @{
 */
#define ST25R3911B_REG_RX_CONF2_PMIX_CL BIT(0)
#define ST25R3911B_REG_RX_CONF2_SQM_DYN BIT(1)
#define ST25R3911B_REG_RX_CONF2_AGC_ALG BIT(2)
#define ST25R3911B_REG_RX_CONF2_AGC_M BIT(3)
#define ST25R3911B_REG_RX_CONF2_AGC_EN BIT(4)
#define ST25R3911B_REG_RX_CONF2_LF_EN BIT(5)
#define ST25R3911B_REG_RX_CONF2_LF_OP BIT(6)
#define ST25R3911B_REG_RX_CONF2_RX_LP BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_rx_conf3_reg Receiver Configuration
 *            Register 3 bits
 * @{
 */
#define ST25R3911B_REG_RX_CONF3_RG_NFC BIT(0)
#define ST25R3911B_REG_RX_CONF3_LIM BIT(1)
#define ST25R3911B_REG_RX_CONF3_RG1_PM0 BIT(2)
#define ST25R3911B_REG_RX_CONF3_RG1_PM1 BIT(3)
#define ST25R3911B_REG_RX_CONF3_RG1_PM2 BIT(4)
#define ST25R3911B_REG_RX_CONF3_RG1_AM0 BIT(5)
#define ST25R3911B_REG_RX_CONF3_RG1_AM1 BIT(6)
#define ST25R3911B_REG_RX_CONF3_RG1_AM2 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_rx_conf4_reg Receiver Configuration
 *            Register 4 bits
 * @{
 */
#define ST25R3911B_REG_RX_CONF4_RG2_PM0 BIT(0)
#define ST25R3911B_REG_RX_CONF4_RG2_PM1 BIT(1)
#define ST25R3911B_REG_RX_CONF4_RG2_PM2 BIT(2)
#define ST25R3911B_REG_RX_CONF4_RG2_PM3 BIT(3)
#define ST25R3911B_REG_RX_CONF4_RG2_AM0 BIT(4)
#define ST25R3911B_REG_RX_CONF4_RG2_AM1 BIT(5)
#define ST25R3911B_REG_RX_CONF4_RG2_AM2 BIT(6)
#define ST25R3911B_REG_RX_CONF4_RG2_AM3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_tim_ctrl_reg General Purpose and
 *            No-Response Timer Control Register bits
 * @{
 */
#define ST25R3911B_REG_TIM_CTRl_NRT_STEP BIT(0)
#define ST25R3911B_REG_TIM_CTRl_NRT_EMV BIT(1)
#define ST25R3911B_REG_TIM_CTRl_GPTC0 BIT(5)
#define ST25R3911B_REG_TIM_CTRl_GPTC1 BIT(6)
#define ST25R3911B_REG_TIM_CTRl_GPTC2 BIT(7)
/**
 * @}
 */

/** @defgroup st25r3911b_fifo_status_2_reg FIFO Status
 *            Register 2 bits
 * @{
 */
#define ST25R3911B_REG_FIFO_STATUS_2_NP_LB BIT(0)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_LB0 BIT(1)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_LB1 BIT(2)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_LB2 BIT(3)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_NCP BIT(4)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_OVR BIT(5)
#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_UNF BIT(6)

#define ST25R3911B_REG_FIFO_STATUS_2_FIFO_LB_MASK (0x0E)

/**
 * @}
 */

/** @defgroup st25r3911b_collision_disp_reg Collision Display
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_COLLISION_DISP_C_PB BIT(0)
#define ST25R3911B_REG_COLLISION_DISP_C_BIT0 BIT(1)
#define ST25R3911B_REG_COLLISION_DISP_C_BIT1 BIT(2)
#define ST25R3911B_REG_COLLISION_DISP_C_BIT2 BIT(3)
#define ST25R3911B_REG_COLLISION_DISP_C_BYTE0 BIT(4)
#define ST25R3911B_REG_COLLISION_DISP_C_BYTE1 BIT(5)
#define ST25R3911B_REG_COLLISION_DISP_C_BYTE2 BIT(6)
#define ST25R3911B_REG_COLLISION_DISP_C_BYTE3 BIT(7)

#define ST25R3911B_REG_COLLISION_DISP_C_BIT_MASK (0x0D)
#define ST25R3911B_REG_COLLISION_DISP_C_BYTE_MASK (0xF0)

/**
 * @}
 */

/** @defgroup st25r3911b_num_tx_bytes_reg Number of Transmitted
 *            Bytes Register 2 bits
 * @{
 */
#define ST25R3911B_REG_NUM_TX_BYTES_REG2_NBTX (0x07)
#define ST25R3911B_REG_NUM_TX_BYTES_REG2_NTX_MASK (0xF8)
#define ST25R3911B_REG_NUM_TX_BYTES_NTX_SHIFT (0x03)
#define ST25R3911B_REG_NUM_TX_BYTES_NTX_SHIFT_LSB (0x05)

/**
 * @}
 */

/** @defgroup st25r3911b_nfcip_bit_rate_reg NFCIP Bit Rate
 *            Detection Display Register bits
 * @{
 */
#define ST25R3911B_REG_NFCIP_BIT_RATE_NFC_RATE0 BIT(4)
#define ST25R3911B_REG_NFCIP_BIT_RATE_NFC_RATE1 BIT(5)
#define ST25R3911B_REG_NFCIP_BIT_RATE_NFC_RATE2 BIT(6)
#define ST25R3911B_REG_NFCIP_BIT_RATE_NFC_RATE3 BIT(7)
/**
 * @}
 */

/** @defgroup st25r3911b_antenna_cal_ctrl_reg Antenna Calibration
 *            Control Register bits
 * @{
 */
#define ST25R3911B_REG_ANTENNA_CAL_CTRL_TRE0 BIT(3)
#define ST25R3911B_REG_ANTENNA_CAL_CTRL_TRE1 BIT(4)
#define ST25R3911B_REG_ANTENNA_CAL_CTRL_TRE2 BIT(5)
#define ST25R3911B_REG_ANTENNA_CAL_CTRL_TRE3 BIT(6)
#define ST25R3911B_REG_ANTENNA_CAL_CTRL_TRIM_S BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_antenna_cal_display_reg Antenna Calibration
 *            Display Register bits
 * @{
 */
#define ST25R3911B_REG_ANTENNA_CAL_DISP_TRI_ERR BIT(3)
#define ST25R3911B_REG_ANTENNA_CAL_DISP_TRI0 BIT(4)
#define ST25R3911B_REG_ANTENNA_CAL_DISP_TRI1 BIT(5)
#define ST25R3911B_REG_ANTENNA_CAL_DISP_TRI2 BIT(6)
#define ST25R3911B_REG_ANTENNA_CAL_DISP_TRI3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_am_mod_depth_ctrl_reg AM Modulation
 *            Depth Control Register bits
 * @{
 */
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD0 BIT(1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD1 BIT(2)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD2 BIT(3)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD3 BIT(4)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD4 BIT(5)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_MOD5 BIT(6)
#define ST25R3911B_REG_AM_MOD_DEPTH_CTRL_AM_S BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_rfo_am_lvl_reg RFO AM Modulated Level
 *            Definition Register bits
 * @{
 */
#define ST25R3911B_REG_RFO_AM_LVL_DRAM0 BIT(0)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM1 BIT(1)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM2 BIT(2)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM3 BIT(3)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM4 BIT(4)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM5 BIT(5)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM6 BIT(6)
#define ST25R3911B_REG_RFO_AM_LVL_DRAM7 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_field_threshold_reg External Field
 *            Threshold Register bits
 * @{
 */
#define ST25R3911B_REG_FIELD_THRESHOLD_RFE_TO BIT(0)
#define ST25R3911B_REG_FIELD_THRESHOLD_RTE_T1 BIT(1)
#define ST25R3911B_REG_FIELD_THRESHOLD_RTE_T2 BIT(2)
#define ST25R3911B_REG_FIELD_THRESHOLD_RTE_T3 BIT(3)
#define ST25R3911B_REG_FIELD_THRESHOLD_TRG_IO BIT(4)
#define ST25R3911B_REG_FIELD_THRESHOLD_TRG_I1 BIT(5)
#define ST25R3911B_REG_FIELD_THRESHOLD_TRG_I2 BIT(6)
#define ST25R3911B_REG_FIELD_THRESHOLD_RFE_MASK (0x0F)
#define ST25R3911B_REG_FIELD_THRESHOLD_TRG_MASK (0xF0)

/**
 * @}
 */

/** @defgroup st25r3911b_regulator_ctrl_reg Regulator Voltage
 *            Control Register bits
 * @{
 */
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV0 BIT(1)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV1 BIT(2)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV_MASK (3 << 1)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV_VDD (0 << 1)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV_VSP_A (1 << 1)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV_VSP_D (2 << 1)
#define ST25R3911B_REG_REGULATOR_CTRL_MPSV_VSP_RF (3 << 1)
#define ST25R3911B_REG_REGULATOR_CTRL_REGE0 BIT(3)
#define ST25R3911B_REG_REGULATOR_CTRL_REGE1 BIT(4)
#define ST25R3911B_REG_REGULATOR_CTRL_REGE2 BIT(5)
#define ST25R3911B_REG_REGULATOR_CTRL_REGE3 BIT(6)
#define ST25R3911B_REG_REGULATOR_CTRL_REG_S BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_regulator_tim_displ_reg Regulator and
 *            Timer Display Register bits
 * @{
 */
#define ST25R3911B_REG_REGULATOR_TIM_DISP_MRT_ON BIT(0)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_NRT_ON BIT(1)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_GPT_ON BIT(2)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_REG_0 BIT(4)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_REG_1 BIT(5)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_REG_2 BIT(6)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_REG_3 BIT(7)
#define ST25R3911B_REG_REGULATOR_TIM_DISP_REG_POS (4)

/**
 * @}
 */

/** @defgroup st25r3911b_rssi_disp_reg RSSI Display Register bits
 * @{
 */
#define ST25R3911B_REG_RSSI_DISP_RSSI_PM0 BIT(0)
#define ST25R3911B_REG_RSSI_DISP_RSSI_PM1 BIT(1)
#define ST25R3911B_REG_RSSI_DISP_RSSI_PM2 BIT(2)
#define ST25R3911B_REG_RSSI_DISP_RSSI_PM3 BIT(3)
#define ST25R3911B_REG_RSSI_DISP_RSSI_AM0 BIT(4)
#define ST25R3911B_REG_RSSI_DISP_RSSI_AM1 BIT(5)
#define ST25R3911B_REG_RSSI_DISP_RSSI_AM2 BIT(6)
#define ST25R3911B_REG_RSSI_DISP_RSSI_AM3 BIT(7)
/**
 * @}
 */

/** @defgroup st25r3911b_gain_reduction_state_reg Gain Reduction
 *            State Register bits
 * @{
 */
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_PM0 BIT(0)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_PM1 BIT(1)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_PM2 BIT(2)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_PM3 BIT(3)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_AM0 BIT(4)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_AM1 BIT(5)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_AM2 BIT(6)
#define ST25R3911B_REG_GAIN_REDUCTION_STATE_GS_AM3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_cap_sensor_ctrl_reg Capacitive Sensor
 *            Control Register bits
 * @{
 */
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_G0 BIT(0)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_G1 BIT(1)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_G2 BIT(2)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_MCAL0 BIT(3)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_MCAL1 BIT(4)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_MCAL2 BIT(5)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_MCAL3 BIT(6)
#define ST25R3911B_REG_CAP_SENSOR_CTRL_CS_MACAL4 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_cap_sensor_disp_reg Capacitive Sensor
 *            Display Register bits
 * @{
 */
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL_ERR BIT(1)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL_END BIT(2)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL0 BIT(3)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL1 BIT(4)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL2 BIT(5)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL3 BIT(6)
#define ST25R3911B_REG_CAP_SENSOR_DISP_CS_CAL4 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_auxiliary_disp_reg Auxiliary Display
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_AUXILIARY_DISP_EN_AC BIT(0)
#define ST25R3911B_REG_AUXILIARY_DISP_NFC_T BIT(1)
#define ST25R3911B_REG_AUXILIARY_DISP_RX_ACT BIT(2)
#define ST25R3911B_REG_AUXILIARY_DISP_RX_ON BIT(3)
#define ST25R3911B_REG_AUXILIARY_DISP_OSC_OK BIT(4)
#define ST25R3911B_REG_AUXILIARY_DISP_TX_ON BIT(5)
#define ST25R3911B_REG_AUXILIARY_DISP_EFD_O BIT(6)
#define ST25R3911B_REG_AUXILIARY_DISP_A_CHA BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_wake_up_tim_reg Wake-Up Timer Control
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_WAKE_UP_TIM_WCAP BIT(0)
#define ST25R3911B_REG_WAKE_UP_TIM_WPH BIT(1)
#define ST25R3911B_REG_WAKE_UP_TIM_WAM BIT(2)
#define ST25R3911B_REG_WAKE_UP_TIM_WTO BIT(3)
#define ST25R3911B_REG_WAKE_UP_TIM_WUT0 BIT(4)
#define ST25R3911B_REG_WAKE_UP_TIM_WUT1 BIT(5)
#define ST25R3911B_REG_WAKE_UP_TIM_WUT2 BIT(6)
#define ST25R3911B_REG_WAKE_UP_TIM_WUR BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_amp_measure_conf_reg Amplitude Measurement
 *            Configuration Register bits
 * @{
 */
#define ST25R3911B_REG_AMP_MEASURE_CONF_AM_AE BIT(0)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AEW2 BIT(1)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AEW1 BIT(2)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AAM BIT(3)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AM_DO BIT(4)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AM_D1 BIT(5)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AM_D2 BIT(6)
#define ST25R3911B_REG_AMP_MEASURE_CONF_AM_D3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_conf_pm_reg Phase Measurement
 *            Configuration Register bits
 * @{
 */
#define ST25R3911B_REG_CONF_PM_AE BIT(0)
#define ST25R3911B_REG_CONF_PM_AEW0 BIT(1)
#define ST25R3911B_REG_CONF_PM_AEW1 BIT(2)
#define ST25R3911B_REG_CONF_PM_AAM BIT(3)
#define ST25R3911B_REG_CONF_PM_D0 BIT(4)
#define ST25R3911B_REG_CONF_PM_D1 BIT(5)
#define ST25R3911B_REG_CONF_PM_D2 BIT(6)
#define ST25R3911B_REG_CONF_PM_D3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_conf_cm_reg Capacitance Measurement
 *            Configuration Register bits
 * @{
 */
#define ST25R3911B_REG_CONF_CM_AE BIT(0)
#define ST25R3911B_REG_CONF_CM_AEW0 BIT(1)
#define ST25R3911B_REG_CONF_CM_AEW1 BIT(2)
#define ST25R3911B_REG_CONF_CM_AAM BIT(3)
#define ST25R3911B_REG_CONF_CM_CM_DO BIT(4)
#define ST25R3911B_REG_CONF_CM_CM_D1 BIT(5)
#define ST25R3911B_REG_CONF_CM_CM_D2 BIT(6)
#define ST25R3911B_REG_CONF_CM_CM_D3 BIT(7)

/**
 * @}
 */

/** @defgroup st25r3911b_ic_identity_reg IC Identity
 *            Register bits
 * @{
 */
#define ST25R3911B_REG_IC_IDENTITY_IC_REV BIT(0)
#define ST25R3911B_REG_IC_IDENTITY_IC_REV_MASK (0x03)
#define ST25R3911B_REG_IC_IDENTITY_IC_TYPE BIT(3)
#define ST25R3911B_REG_IC_IDENTITY_IC_TYPE_MASK (0xF1)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ST25R3911B_REG_H_ */
