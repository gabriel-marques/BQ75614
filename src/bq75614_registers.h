#ifndef BQ75614_REG_H_
#define BQ75614_REG_H_

// OTP registers

#define REG_DIR0_ADDR_OTP 0x0
#define REG_DIR1_ADDR_OTP 0x1
#define REG_DEV_CONF 0x2
#define REG_ACTIVE_CELL 0x3
#define REG_OTP_SPARE15 0x4
#define REG_ADC_CONF1 0x7
#define REG_ADC_CONF2 0x8
#define REG_OV_THRESH 0x9
#define REG_UV_THRESH 0xA
#define REG_OTUT_THRESH 0xB
#define REG_UV_DISABLE1 0xC
#define REG_UV_DISABLE2 0xD
#define REG_GPIO_CONF1 0xE
#define REG_GPIO_CONF2 0xF
#define REG_GPIO_CONF3 0x10
#define REG_GPIO_CONF4 0x11
#define REG_OTP_SPARE14 0x12
#define REG_OTP_SPARE13 0x13
#define REG_OTP_SPARE12 0x14
#define REG_OTP_SPARE11 0x15
#define REG_FAULT_MSK1 0x16
#define REG_FAULT_MSK2 0x17
#define REG_PWR_TRANSIT_CONF 0x18
#define REG_COMM_TIMEOUT_CONF 0x19
#define REG_TX_HOLD_OFF 0x1A
#define REG_MAIN_ADC_CAL1 0x1B
#define REG_MAIN_ADC_CAL2 0x1C
#define REG_AUX_ADC_CAL1 0x1D
#define REG_AUX_ADC_CAL2 0x1E
#define REG_CS_ADC_CAL1 0x1F
#define REG_CS_ADC_CAL2 0x20
#define REG_CUST_MISC1 0x21
#define REG_CUST_MISC2 0x22
#define REG_CUST_MISC3 0x23
#define REG_CUST_MISC4 0x24
#define REG_CUST_MISC5 0x25
#define REG_CUST_MISC6 0x26
#define REG_CUST_MISC7 0x27
#define REG_CUST_MISC8 0x28
#define REG_OTP_SPARE10 0x2C
#define REG_OTP_SPARE9 0x2D
#define REG_OTP_SPARE8 0x2E
#define REG_OTP_SPARE7 0x2F
#define REG_OTP_SPARE6 0x30
#define REG_OTP_SPARE5 0x31
#define REG_OTP_SPARE4 0x32
#define REG_OTP_SPARE3 0x33
#define REG_OTP_SPARE2 0x34
#define REG_OTP_SPARE1 0x35
#define REG_CUST_CRC_HI 0x36
#define REG_CUST_CRC_LO 0x37

// Read/Write registers

#define REG_OTP_PROG_UNLOCK1A 0x300
#define REG_OTP_PROG_UNLOCK1B 0x301
#define REG_OTP_PROG_UNLOCK1C 0x302
#define REG_OTP_PROG_UNLOCK1D 0x303
#define REG_DIR0_ADDR 0x306
#define REG_DIR1_ADDR 0x307
#define REG_COMM_CTRL 0x308
#define REG_CONTROL1 0x309
#define REG_CONTROL2 0x30A
#define REG_OTP_PROG_CTRL 0x30B
#define REG_ADC_CTRL1 0x30D
#define REG_ADC_CTRL2 0x30E
#define REG_ADC_CTRL3 0x30F
#define REG_CB_CELL16_CTRL 0x318
#define REG_CB_CELL15_CTRL 0x319
#define REG_CB_CELL14_CTRL 0x31A
#define REG_CB_CELL13_CTRL 0x31B
#define REG_CB_CELL12_CTRL 0x31C
#define REG_CB_CELL11_CTRL 0x31D
#define REG_CB_CELL10_CTRL 0x31E
#define REG_CB_CELL9_CTRL 0x31F
#define REG_CB_CELL8_CTRL 0x320
#define REG_CB_CELL7_CTRL 0x321
#define REG_CB_CELL6_CTRL 0x322
#define REG_CB_CELL5_CTRL 0x323
#define REG_CB_CELL4_CTRL 0x324
#define REG_CB_CELL3_CTRL 0x325
#define REG_CB_CELL2_CTRL 0x326
#define REG_CB_CELL1_CTRL 0x327
#define REG_VCB_DONE_THRESH 0x32A
#define REG_OTCB_THRESH 0x32B
#define REG_OVUV_CTRL 0x32C
#define REG_OTUT_CTRL 0x32D
#define REG_BAL_CTRL1 0x32E
#define REG_BAL_CTRL2 0x32F
#define REG_BAL_CTRL3 0x330
#define REG_FAULT_RST1 0x331
#define REG_FAULT_RST2 0x332
#define REG_DIAG_OTP_CTRL 0x335
#define REG_DIAG_COMM_CTRL 0x336
#define REG_DIAG_PWR_CTRL 0x337
#define REG_DIAG_CBFET_CTRL1 0x338
#define REG_DIAG_CBFET_CTRL2 0x339
#define REG_DIAG_COMP_CTRL1 0x33A
#define REG_DIAG_COMP_CTRL2 0x33B
#define REG_DIAG_COMP_CTRL3 0x33C
#define REG_DIAG_COMP_CTRL4 0x33D
#define REG_DIAG_PROT_CTRL 0x33E
#define REG_OTP_ECC_DATAIN1 0x343
#define REG_OTP_ECC_DATAIN2 0x344
#define REG_OTP_ECC_DATAIN3 0x345
#define REG_OTP_ECC_DATAIN4 0x346
#define REG_OTP_ECC_DATAIN5 0x347
#define REG_OTP_ECC_DATAIN6 0x348
#define REG_OTP_ECC_DATAIN7 0x349
#define REG_OTP_ECC_DATAIN8 0x34A
#define REG_OTP_ECC_DATAIN9 0x34B
#define REG_OTP_ECC_TEST 0x34C
#define REG_SPI_CONF 0x34D
#define REG_SPI_TX3 0x34E
#define REG_SPI_TX2 0x34F
#define REG_SPI_TX1 0x350
#define REG_SPI_EXE 0x351
#define REG_OTP_PROG_UNLOCK2A 0x352
#define REG_OTP_PROG_UNLOCK2B 0x353
#define REG_OTP_PROG_UNLOCK2C 0x354
#define REG_OTP_PROG_UNLOCK2D 0x355

// Read REG_only registers

#define REG_PARTID 0x500
#define REG_DEV_REVID 0xE00
#define REG_DIE_ID1 0x501
#define REG_DIE_ID2 0x502
#define REG_DIE_ID3 0x503
#define REG_DIE_ID4 0x504
#define REG_DIE_ID5 0x505
#define REG_DIE_ID6 0x506
#define REG_DIE_ID7 0x507
#define REG_DIE_ID8 0x508
#define REG_DIE_ID9 0x509
#define REG_CUST_CRC_RSLT_HI 0x50C
#define REG_CUST_CRC_RSLT_LO 0x50D
#define REG_OTP_ECC_DATAOUT1 0x510
#define REG_OTP_ECC_DATAOUT2 0x511
#define REG_OTP_ECC_DATAOUT3 0x512
#define REG_OTP_ECC_DATAOUT4 0x513
#define REG_OTP_ECC_DATAOUT5 0x514
#define REG_OTP_ECC_DATAOUT6 0x515
#define REG_OTP_ECC_DATAOUT7 0x516
#define REG_OTP_ECC_DATAOUT8 0x517
#define REG_OTP_ECC_DATAOUT9 0x518
#define REG_OTP_PROG_STAT 0x519
#define REG_OTP_CUST1_STAT 0x51A
#define REG_OTP_CUST2_STAT 0x51B
#define REG_SPI_RX3 0x520
#define REG_SPI_RX2 0x521
#define REG_SPI_RX1 0x522
#define REG_DIAG_STAT 0x526
#define REG_ADC_STAT1 0x527
#define REG_ADC_STAT2 0x528
#define REG_GPIO_STAT 0x52A
#define REG_BAL_STAT 0x52B
#define REG_DEV_STAT 0x52C
#define REG_FAULT_SUMMARY 0x52D
#define REG_FAULT_COMM1 0x530
#define REG_FAULT_OTP 0x535
#define REG_FAULT_SYS 0x536
#define REG_FAULT_PROT1 0x53A
#define REG_FAULT_PROT2 0x53B
#define REG_FAULT_OV1 0x53C
#define REG_FAULT_OV2 0x53D
#define REG_FAULT_UV1 0x53E
#define REG_FAULT_UV2 0x53F
#define REG_FAULT_OT 0x540
#define REG_FAULT_UT 0x541
#define REG_FAULT_COMP_GPIO 0x543
#define REG_FAULT_COMP_VCCB1 0x545
#define REG_FAULT_COMP_VCCB2 0x546
#define REG_FAULT_COMP_VCOW1 0x548
#define REG_FAULT_COMP_VCOW2 0x549
#define REG_FAULT_COMP_CBOW1 0x54B
#define REG_FAULT_COMP_CBOW2 0x54C
#define REG_FAULT_COMP_CBFET1 0x54E
#define REG_FAULT_COMP_CBFET2 0x54F
#define REG_FAULT_COMP_MISC 0x550
#define REG_FAULT_PWR1 0x552
#define REG_FAULT_PWR2 0x553
#define REG_FAULT_PWR3 0x554
#define REG_CB_COMPLETE1 0x556
#define REG_CB_COMPLETE2 0x557
#define REG_BAL_TIME 0x558
#define REG_VCELL16_HI 0x568
#define REG_VCELL16_LO 0x569
#define REG_VCELL15_HI 0x56A
#define REG_VCELL15_LO 0x56B
#define REG_VCELL14_HI 0x56C
#define REG_VCELL14_LO 0x56D
#define REG_VCELL13_HI 0x56E
#define REG_VCELL13_LO 0x56F
#define REG_VCELL12_HI 0x570
#define REG_VCELL12_LO 0x571
#define REG_VCELL11_HI 0x572
#define REG_VCELL11_LO 0x573
#define REG_VCELL10_HI 0x574
#define REG_VCELL10_LO 0x575
#define REG_VCELL9_HI 0x576
#define REG_VCELL9_LO 0x577
#define REG_VCELL8_HI 0x578
#define REG_VCELL8_LO 0x579
#define REG_VCELL7_HI 0x57A
#define REG_VCELL7_LO 0x57B
#define REG_VCELL6_HI 0x57C
#define REG_VCELL6_LO 0x57D
#define REG_VCELL5_HI 0x57E
#define REG_VCELL5_LO 0x57F
#define REG_VCELL4_HI 0x580
#define REG_VCELL4_LO 0x581
#define REG_VCELL3_HI 0x582
#define REG_VCELL3_LO 0x583
#define REG_VCELL2_HI 0x584
#define REG_VCELL2_LO 0x585
#define REG_VCELL1_HI 0x586
#define REG_VCELL1_LO 0x587
#define REG_MAIN_CURRENT_HI 0x588
#define REG_MAIN_CURRENT_LO 0x589
#define REG_TSREF_HI 0x58C
#define REG_TSREF_LO 0x58D
#define REG_GPIO1_HI 0x58E
#define REG_GPIO1_LO 0x58F
#define REG_GPIO2_HI 0x590
#define REG_GPIO2_LO 0x591
#define REG_GPIO3_HI 0x592
#define REG_GPIO3_LO 0x593
#define REG_GPIO4_HI 0x594
#define REG_GPIO4_LO 0x595
#define REG_GPIO5_HI 0x596
#define REG_GPIO5_LO 0x597
#define REG_GPIO6_HI 0x598
#define REG_GPIO6_LO 0x599
#define REG_GPIO7_HI 0x59A
#define REG_GPIO7_LO 0x59B
#define REG_GPIO8_HI 0x59C
#define REG_GPIO8_LO 0x59D
#define REG_DIETEMP1_HI 0x5AE
#define REG_DIETEMP1_LO 0x5AF
#define REG_DIETEMP2_HI 0x5B0
#define REG_DIETEMP2_LO 0x5B1
#define REG_AUX_CELL_HI 0x5B2
#define REG_AUX_CELL_LO 0x5B3
#define REG_AUX_GPIO_HI 0x5B4
#define REG_AUX_GPIO_LO 0x5B5
#define REG_AUX_BAT_HI 0x5B6
#define REG_AUX_BAT_LO 0x5B7
#define REG_AUX_REFL_HI 0x5B8
#define REG_AUX_REFL_LO 0x5B9
#define REG_AUX_VBG2_HI 0x5BA
#define REG_AUX_VBG2_LO 0x5BB
#define REG_AUX_AVAO_REF_HI 0x5BE
#define REG_AUX_AVAO_REF_LO 0x5BF
#define REG_AUX_AVDD_REF_HI 0x5C0
#define REG_AUX_AVDD_REF_LO 0x5C1
#define REG_AUX_OV_DAC_HI 0x5C2
#define REG_AUX_OV_DAC_LO 0x5C3
#define REG_AUX_UV_DAC_HI 0x5C4
#define REG_AUX_UV_DAC_LO 0x5C5
#define REG_AUX_OT_OTCB_DAC_HI 0x5C6
#define REG_AUX_OT_OTCB_DAC_LO 0x5C7
#define REG_AUX_UT_DAC_HI 0x5C8
#define REG_AUX_UT_DAC_LO 0x5C9
#define REG_AUX_VCBDONE_DAC_HI 0x5CA
#define REG_AUX_VCBDONE_DAC_LO 0x5CB
#define REG_AUX_VCM_HI 0x5CC
#define REG_AUX_VCM_LO 0x5CD
#define REG_REFOVDAC_HI 0x5D0
#define REG_REFOVDAC_LO 0x5D1
#define REG_DIAG_MAIN_HI 0x5D2
#define REG_DIAG_MAIN_LO 0x5D3
#define REG_DIAG_AUX_HI 0x5D4
#define REG_DIAG_AUX_LO 0x5D5
#define REG_CURRENT_HI 0x5D6
#define REG_CURRENT_MID 0x5D7
#define REG_CURRENT_LO 0x5D8
#define REG_DEBUG_UART_RC 0x781
#define REG_DEBUG_UART_RR_TR 0x782
#define REG_DEBUG_UART_DISCARD 0x789
#define REG_DEBUG_UART_VALID_HI 0x78C
#define REG_DEBUG_UART_VALID_LO 0x78D
#define REG_DEBUG_OTP_SEC_BLK 0x7A0
#define REG_DEBUG_OTP_DED_BLK 0x7A1

// Registers bit position

// DEV_CONF register bit position

#define NFAULT_EN_POS 2   // Position of bit that enables the NFAULT function in DEV_CONF register
#define TWO_STOP_EN_POS 3 // Position of bit that enables the two stop bits for UART in DEV_CONF register
#define FCOMM_EN_POS 4    // Position of bit that enables the fault state detection through communication in ACTIVE mode in DEV_CONF register
#define NO_ADJ_CB_POS 6   // Read the documentation for this bit chap. 8.5.4.3.1 of datasheet

// FAULT_SUMMARY register bit position

#define BQ75614_FAULT_PROT_POS 7
#define BQ75614_FAULT_COMP_ADC_POS 6
#define BQ75614_FAULT_OTP_POS 5
#define BQ75614_FAULT_COMM_POS 4
#define BQ75614_FAULT_OTUT_POS 3
#define BQ75614_FAULT_OVUV_POS 2
#define BQ75614_FAULT_SYS_POS 1
#define BQ75614_FAULT_PWR_POS 0

// CONTROL1 register bit position

#define CONTROL1_SOFT_RESET_POS 1
#define CONTROL1_SLEEP_POS 2
#define CONTROL1_SHUTDOWN_POS 3

// OTP_CUST*_STAT register bit position

#define OTP_CUST_STAT_TRY_POS 0     // Indicates that the OTP data has been tried to be programmed
#define OTP_CUST_STAT_OVOK_POS 1    // Indicates an overvoltage condition when programming the OTP data, see documentation
#define OTP_CUST_STAT_UVOK_POS 2    // Indicates an undervoltage condition when programming the OTP data, see documentation
#define OTP_CUST_STAT_PROGOK_POS 3  // Indicates that the OTP data has been programmed successfully
#define OTP_CUST_STAT_FMTERR_POS 4  // Indicates an error during the OTP data formatting. see documentation
#define OTP_CUST_STAT_LOADERR_POS 5 // Indicates an error during the OTP data loading, DED was detected (Double Error Detection)
#define OTP_CUST_STAT_LOADWRN_POS 6 // Indicates that the OTP data has been loaded with one or more SEC warnings (Single Error Correction)
#define OTP_CUST_STAT_LOADED_POS 7  // Indicates that the OTP data has been loaded

// OTP_PROG_STAT register bit position

#define OTP_PROG_STAT_DONE_POS 0    // Indicates that the OTP data has been programmed
#define OTP_PROG_STAT_PROGERR_POS 1 // Indicates an error during the OTP data programming. See documentation
#define OTP_PROG_STAT_SOVERR_POS 2  // Indicates an overvoltage condition during the voltage stability test
#define OTP_PROG_STAT_SUVERR_POS 3  // Indicates an undervoltage condition during the voltage stability test
#define OTP_PROG_STAT_OVERR_POS 4   // Indicates an overvoltage condition during the OTP data programming. See documentation
#define OTP_PROG_STAT_UVERR_POS 5   // Indicates an undervoltage condition during the OTP data programming. See documentation
#define OTP_PROG_STAT_OTERR_POS 6   // Indicates an overtemperature condition during the OTP data programming. See documentation
#define OTP_PROG_STAT_UNLOCK_POS 7  // Indicates that the OTP data has been unlocked

// OTP_PROG_CTRL register bit position

#define OTP_PROG_CTRL_PROG_GO_POS 0 // Start the OTP data programming
#define OTP_PROG_CTRL_PAGESEL_POS 1 // Select the OTP page to be programmed (0 -> page 1 or 1-> page 2)

// OVUV_CTRL register bit position

#define OVUV_CTRL_MODE_POS 0             // Select the mode of the OVUV protection, 00 -> stop, 01 -> round robin run, 10 -> BIST, 11 -> single channel
#define OVUV_CTRL_GO_POS 2               // Start the OVUV protection
#define OVUV_CTRL_LOCK_POS 3             // Lock the OVUV protection to a single channel
#define OVUV_CTRL_VCBDONE_THR_LOCK_POS 7 // Selects which of UV threshold (0) or VCBDONE threshold (1) is used for UV comparator

// OTUT_CTRL register bit position
#define OTUT_CTRL_MODE_POS 0 // Select the mode of the OTUT protection, 00 -> stop, 01 -> round robin run, 10 -> BIST, 11 -> single channel
#define OTUT_CTRL_GO_POS 2   // Start the OTUT protection
#define OTUT_CTRL_LOCK_POS 3 // Lock the OTUT protection to a single channel
#define OTUT_CTRL_OTCB_THR_LOCK_POS 6

// OTUT/OVUV modes

#define PROTECTION_STOP_MODE 0
#define PROTECTION_ROUND_ROBIN_RUN_MODE 1
#define PROTECTION_BIST_MODE 2 // Built-in self-test mode
#define PROTECTION_SINGLE_RUN_MODE 3

// DEV_STAT register bit position

#define DEV_STAT_MAIN_RUN_POS 0      // Status of main ADC
#define DEV_STAT_AUX_RUN_POS 1       // Status of auxiliary ADC
#define DEV_STAT_CS_RUN_POS 2        // Status of current sense ADC
#define DEV_STAT_OVUV_RUN_POS 3      // Status of OVUV protection
#define DEV_STAT_OTUT_RUN_POS 4      // Status of OTUT protection
#define DEV_STAT_CUST_CRC_DONE_POS 5 // Status of the customer CRC state machine
#define DEV_STAT_FACT_CRC_DONE_POS 6 // Status of the factory CRC state machine

// CONTROL2 register bit position

#define CONTROL2_TSREF_EN_POS 0 // Enable the 5V output on the TSREF pin

// PWR_TRANSIT_CONF register bit

#define PWR_TRANSIT_CONF_SLP_TIME_POS 0  // Time after which the device goes from sleep to shutdown
#define PWR_TRANSIT_CONF_TWARN_THR_POS 3 // Threshold for the die temperature warning

// BAL_CTRL1 register bit position
#define BAL_CTRL1_DUTY_POS 0 // Duty cycle of the cell balancing between odd and even cells

// BAL_CTRL2 register bit position
#define BAL_CTRL2_AUTO_BAL_POS 0   // Select the automatic or manual balancing mode
#define BAL_CTRL2_BAL_GO_POS 1     // Enable the cell balancing
#define BAL_CTRL2_BAL_ACT_POS 2    // [3:2] Select the balancing action when cell balancing is done. 00 -> no action, 01 -> sleep, 10 -> shutdown, 11 -> reserved
#define BAL_CTRL2_OTCB_EN_POS 4    // Enable the OTCB detection for cell balancing
#define BAL_CTRL2_FLTSTOP_EN_POS 5 // stops the cell balancing when a fault is detected
#define BAL_CTRL2_CB_PAUSE_POS 6   // Pause the cell balancing

// BAL_CTRL3 register bit position
#define BAL_CTRL3_BAL_TIME_GO_POS 0  // Instruct the device to report the remaining time for cell balancing of cell (BAL_TIME_SEL) into BAL_TIME register
#define BAL_CTRL3_BAL_TIME_SEL_POS 1 // [4:1] Select the cell for which the remaining time for cell balancing is reported

// BAL_STAT register bit position

#define BAL_STAT_CB_DONE_POS 0        // Cell balancing is done
#define BAL_STAT_ABORTFLT_POS 1       // Cell balancing is aborted due to a fault
#define BAL_STAT_CB_RUN_POS 3         // Cell balancing is running
#define BAL_STAT_CB_INPAUSE_POS 5     // Cell balancing is paused
#define BAL_STAT_OT_PAUSE_DET_POS 6   // Overtemperature pause detection
#define BAL_STAT_INVALID_CBCONF_POS 7 // Invalid cell balancing configuration

// BAL_TIME register bit position
#define BAL_TIME_TIME_POS 0      // Remaining time for cell balancing
#define BAL_TIME_TIME_UNIT_POS 7 // Time unit for the remaining time for cell balancing, 0 = sec and 1 = min

// OTCB_THRESH register bit position
#define OTCB_THRESH_POS 0         // Overtemperature threshold for cell balancing
#define OTCB_THRESH_COOLOFF_POS 4 // Cool off ratio for OTCB

// ADC_CTRL1 register bit position
#define ADC_CTRL1_CS_MAIN_MODE_POS 0 // Select the mode of the main current sense ADC
#define ADC_CTRL1_CS_MAIN_GO_POS 2   // Start the main current sense ADC
#define ADC_CTRL1_LPF_VCELL_EN_POS 3 // Enable the low pass filter for the cell voltage ADC
#define ADC_CTRL1_LPF_SR_EN_POS 4    // Enable the low pass filter for the current sense ADC
#define ADC_CTRL1_CS_DR_POS 5        // Select the single measurement time of current sense ADC

// ADC_CTRL2 register bit position
#define ADC_CTRL2_AUX_CELL_SEL_POS 0   // Select which AUXCELL channel will be multiplexed through the AUX ADC
#define ADC_CTRL2_AUX_CELL_ALIGN_POS 3 // Select the alignment of the AUX ADC AUXCELL channel, 0 to Main ADC CELL1 and 1 to Main ADC CELL8
#define ADC_CTRL2_MAINBB_AFE_DIS_POS 4 // Disconnect the main ADC SRP/SRN AFE from SRP/SRN pin. 0 -> connect, 1 -> disconnect

// ADC_CTRL3 register bit position
#define ADC_CTRL3_AUX_MODE_POS 0     // Select the mode of the AUX ADC
#define ADC_CTRL3_AUX_GO_POS 2       // Start the AUX ADC
#define ADC_CTRL3_AUX_GPIO_SEL_POS 3 // Select the GPIO to be multiplexed through the AUX ADC

// ADC_CONF1 register bit position
#define ADC_CONF1_LPF_VCELL_POS 0  // Select the low pass filter for the cell voltage ADC
#define ADC_CONF1_LPF_SR_POS 3     // Select the low pass filter for the current sense MAIN ADC
#define ADC_CONF1_AUX_SETTLE_POS 6 // Select the settling time for the AUX ADC

// ADC_CONF2 register bit position
#define ADC_CONF2_ADC_DLY_POS 0 // Select the delay before the main ADC starts

// COMM_TIMEOUT_CONF register bit position
#define COMM_TIMEOUT_CONF_CTL_TIME_POS 0 // Select the long communication timeout before doing action selected below
#define COMM_TIMEOUT_CONF_CTL_ACT_POS 3  // Select the action to do when the long communication timeout is reached
#define COMM_TIMEOUT_CONF_CTS_TIME_POS 4 // Select the short communication timeout to send a notification to the host through the NFAULT pin

#endif /* BQ75614_REG_H_ */