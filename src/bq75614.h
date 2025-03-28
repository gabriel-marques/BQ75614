/**
 * @details This file contains the prototypes of the functions that can be used to communicate with the BQ75614
 * @details It defines all struct, defines, prototypes and enums that are used in the library
 */
#ifndef _BQ75614_H
#define _BQ75614_H

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "bq75614_defines.h"   // User defined values
#include "bq75614_registers.h" // Register addresses

// Verification that user defined values doesn't exceeds the limits

#define BQ75614_MAX_SHUNT_RESISTOR 1000 // Maximum value for the shunt resistor in mOhm // actually arbitrary value
#define BQ75614_MIN_SHUNT_RESISTOR 1    // Minimum value for the shunt resistor in mOhm

// Compile-time check for shunt resistor value -> Throws an error at compilation time if the value is out of range
#if BQ75614_SHUNT_RESISTOR_USER > 0
static_assert(BQ75614_SHUNT_RESISTOR_USER <= BQ75614_MAX_SHUNT_RESISTOR, "Shunt resistor value exceeds maximum limit.");
static_assert(BQ75614_SHUNT_RESISTOR_USER >= BQ75614_MIN_SHUNT_RESISTOR, "Shunt resistor value exceeds minimum limit.");
#endif

// End of verification

// The following macros MUST be set in the bq75614_defines.h file by user.

#define BQ75614_SHUNT_RESISTOR (float)BQ75614_SHUNT_RESISTOR_USER

// End of user defines

// Constants

// BQ75614 frame sizes on UART
#define BQ75614_FRAME_HEADERS_SIZE (1 + 1 + 2) // Init byte + Device address + Register address
#define BQ75614_FRAME_CRC_SIZE 2               // CRC size
#define BQ75614_MAX_READ_PAYLOAD_SIZE 128      // Maximum bytes we can read from the device
#define BQ75614_MAX_READ_FRAME_SIZE (BQ75614_FRAME_HEADERS_SIZE +    \
                                     BQ75614_MAX_READ_PAYLOAD_SIZE + \
                                     BQ75614_FRAME_CRC_SIZE) // Maximum size of READ frame
#define BQ75614_MAX_WRITE_PAYLOAD_SIZE 8                     // Maximum bytes we can write to the device
#define BQ75614_MAX_WRITE_FRAME_SIZE (BQ75614_FRAME_HEADERS_SIZE +     \
                                      BQ75614_MAX_WRITE_PAYLOAD_SIZE + \
                                      BQ75614_FRAME_CRC_SIZE) // Maximum size of WRITE frame

#define BQ75614_UART_TIMEOUT 100 // Time in ms to wait for UART response

// Frame type and request type bit position -> see SLUSDT5B, chapter "8.3.6.1.1.2.1 Transaction Frame Structure" for more details

#define FRAME_TYPE_POS 7     // Position of the bit in the frame init byte that indicates if it's a command or a response
#define FRAME_REQ_TYPE_POS 4 // Position of the bit in the frame init byte that indicates the request type

#define FRAME_INIT_BYTE_INDEX 0      // Index of the init byte in the frame
#define FRAME_DEVICE_ADDRESS_INDEX 1 // Index of the device address in the frame
#define FRAME_REG_ADDRESS_INDEX_1 2  // Index of the first byte of the register address in the frame
#define FRAME_REG_ADDRESS_INDEX_2 3  // Index of the second byte of the register address in the frame
#define FRAME_DATA_BASE_INDEX 4      // Index of the first byte of the data in the frame

// Constants from datasheet
#define BQ75614_MIN_ACTIVE_CELLS 6  // Minimum number of active cells
#define BQ75614_MAX_ACTIVE_CELLS 16 // Maximum number of active cells

#define BQ75614_MAX_OT_RATIO 39 // Maximum ratio for overtemperature threshold
#define BQ75614_MAX_UT_RATIO 80 // Maximum ratio for undertamperature threshold
#define BQ75614_MIN_OT_RATIO 10 // Minimum ratio for overtamperature threshold
#define BQ75614_MIN_UT_RATIO 66 // Minimum ratio for undertamperature threshold

#define BQ75614_MIN_VCB_DONE_THRESHOLD 2450 // Minimum threshold for VCBDONE in mV
#define BQ75614_MAX_VCB_DONE_THRESHOLD 4000 // Maximum threshold for VCBDONE in mV

#define BQ75614_MIN_OTCB_COOLOFF_RATIO 4  // Minimum ratio for OTCB cool off
#define BQ75614_MAX_OTCB_COOLOFF_RATIO 14 // Maximum ratio for OTCB cool off

#define BQ75614_MIN_OTCB_THRESHOLD 10 // Minimum ratio for OTCB threshold
#define BQ75614_MAX_OTCB_THRESHOLD 24 // Maximum ratio for OTCB threshold

#define BQ75614_ADC_RESOLUTION 192.73f                // uV/LSB
#define BQ75614_ADC_DIE_TEMPERATURE_RESOLUTION 0.025f // °C/LSB
#define BQ75614_ADC_MAIN_CURRENT_RESOLUTION 30.52f    // uV/LSB
#define BQ75614_ADC_CS_CURRENT_RESOLUTION 0.0149f     // uV/LSB
#define BQ75614_ADC_TSREF_RESOLUTION 169.54f          // uV/LSB
#define BQ75614_ADC_GPIO_RESOLUTION 152.59f           // uV/LSB
#define BQ75614_ADC_AUX_BAT_RESOLUTION 3050.0f        // uV/LSB

#define BQ75614_BAL_TIME_RESOLUTIONS_STEP 5 // Time resolution for cell balancing in seconds or minutes

#define OTP_PROG_TIME 1000 // Time in ms to wait for OTP programming // TODO: Check this value in datasheet

#define BQ75614_PARTID 0x03               // Part ID of the BQ75614 -> if different, it means that the device is not a BQ75614, not awake or a a sample
#define BQ75614_NBR_OF_FAULT_REGISTERS 25 // Number of fault registers, including the fault summary

/**
 * @brief State of the BQ75614 device
 */
typedef enum BQ75614_DeviceStateType
{
    BQ75614_STATE_ACTIVE = 0,
    BQ75614_STATE_SLEEP = 1,
    BQ75614_STATE_SHUTDOWN = 2,
} BQ75614_DeviceStateType;

/**
 * @brief Structure that contains the state and address of the BQ75614 device
 * @note The device address is not used because the device is standalone
 * @note This structure can be extended with more parameters if needed
 */
typedef struct BQ75614_HandleTypeDef
{
    uint8_t device_address;        // Not used because device is standalone
    BQ75614_DeviceStateType state; // Actual state of the device, ACTIVE, SLEEP or SHUTDOWN. /!\ real state can be different if an error occurred on BQ75614
} BQ75614_HandleTypeDef;

/**
 * @brief Enum that contains the different GPIO configuration
 */
typedef enum BQ75614_GPIOConf
{
    BQ75614_GPIO_DISABLE = 0,           // Default state
    BQ75614_GPIO_INPUT_ADC_OTUT = 1,    // Read an ADC input or a thermistor for OTUT protection
    BQ75614_GPIO_INPUT_ADC = 2,         // Read an ADC input only
    BQ75614_GPIO_INPUT_DIGITAL = 3,     // Reads a digital input and result is available at GPIO_STAT register
    BQ75614_GPIO_OUTPUT_HIGH = 4,       // Set the GPIO to high
    BQ75614_GPIO_OUTPUT_LOW = 5,        // Set the GPIO to low
    BQ75614_GPIO_ADC_WEAK_PULLUP = 6,   // Read an ADC input with a weak pullup
    BQ75614_GPIO_ADC_WEAK_PULLDOWN = 7, // Read an ADC input with a weak pulldown
} BQ75614_GPIOConf;

/**
 * @brief Enum of the different GPIOs available on the BQ75614
 */
typedef enum BQ75614_GPIO
{
    BQ75614_GPIO_1 = (1 << 0),
    BQ75614_GPIO_2 = (1 << 1),
    BQ75614_GPIO_3 = (1 << 2),
    BQ75614_GPIO_4 = (1 << 3),
    BQ75614_GPIO_5 = (1 << 4),
    BQ75614_GPIO_6 = (1 << 5),
    BQ75614_GPIO_7 = (1 << 6),
    BQ75614_GPIO_8 = (1 << 7),
} BQ75614_GPIO;

/**
 * @brief Enum of the different Battery Cells available on the BQ75614
 */
typedef enum BQ75614_Cell
{
    BQ75614_CELL1 = (1 << 0),
    BQ75614_CELL2 = (1 << 1),
    BQ75614_CELL3 = (1 << 2),
    BQ75614_CELL4 = (1 << 3),
    BQ75614_CELL5 = (1 << 4),
    BQ75614_CELL6 = (1 << 5),
    BQ75614_CELL7 = (1 << 6),
    BQ75614_CELL8 = (1 << 7),
    BQ75614_CELL9 = (1 << 8),
    BQ75614_CELL10 = (1 << 9),
    BQ75614_CELL11 = (1 << 10),
    BQ75614_CELL12 = (1 << 11),
    BQ75614_CELL13 = (1 << 12),
    BQ75614_CELL14 = (1 << 13),
    BQ75614_CELL15 = (1 << 14),
    BQ75614_CELL16 = (1 << 15),
} BQ75614_Cell;

/**
 * @brief Enum of the different errors that lib can return
 */
typedef enum BQ75614_StatusType
{
    BQ75614_OK = 0,
    BQ75614_ERROR = 1,
    BQ75614_ERROR_BAD_CRC = 2,
    BQ75614_ERROR_BAD_DEVICE = 3,
    BQ75614_ERROR_BAD_REGISTER = 4,
    BQ75614_ERROR_REGISTER_NOT_SET = 5,
    BQ75614_ERROR_BAD_PARAMETER = 6,
    BQ75614_ERROR_OTP_NOT_READY = 7,
    BQ75614_ERROR_OTP_UNLOCK_FAILED = 8,
    BQ75614_ERROR_OTP_PROGRAMMING_FAILED = 9,
    BQ75614_ERROR_UART_RECEIVE_FAILED = 10,
    BQ75614_ERROR_UART_TRANSMIT_FAILED = 11,
    BQ75614_ERROR_OVUV_NOT_RUNNING = 12,
    BQ75614_ERROR_OTUT_NOT_RUNNING = 13,
    BQ75614_ERROR_TSREF_NOT_ENABLED = 14,
    BQ75614_ERROR_BALANCING_NOT_RUNNING = 15,
    BQ75614_ERROR_BALANCING_CONF_INVALID = 16,
    BQ75614_ERROR_CS_CURRENT_OUT_OF_RANGE = 17,
    BQ75614_ERROR_BAD_STATE = 18,
    BQ75614_ERROR_MAIN_ADC_NOT_RUNNING = 20,
    BQ75614_ERROR_NULL_POINTER = 21,
    BQ75614_ERROR_NOT_IMPLEMENTED = 22,
    BQ75614_ERROR_BAD_PARTID = 23,
    BQ75614_ERROR_REPLACE_DEVICE = 24,
} BQ75614_StatusType;

/**
 * @brief Type of fault to mask or unmask. Value is the bit position in the MASK_FAULT1/2 register.
 * @note Bit positions under 8 are in MASK_FAULT1 register, above are in MASK_FAULT2 register.
 * @note This is managed by lib, don't worry about it.
 */
typedef enum BQ75614_MaskFaultType
{
    MASK_PWR = (1 << 0),
    MASK_SYS = (1 << 1),
    MASK_COMP = (1 << 2),
    MASK_OV = (1 << 3),
    MASK_UV = (1 << 4),
    MASK_OT = (1 << 5),
    MASK_UT = (1 << 6),
    MASK_PROT = (1 << 7),
    MASK_COMM1 = (1 << 8),
    MASK_OTP_DATA = (1 << 13),
    MASK_OTP_CRC = (1 << 14),
} BQ75614_MaskFaultType;

/**
 * @brief Type of fault to check. Value is the address of matching registers.
 */
typedef enum BQ75614_FaultType
{
    // Fault present in FAULT_SUMMARY register
    FAULT_SUMMARY = REG_FAULT_SUMMARY,

    // Fault present in lower register of FAULT_PROT
    FAULT_PROT_1 = REG_FAULT_PROT1,
    FAULT_PROT_2 = REG_FAULT_PROT2,

    // Fault present in lower register of FAULT_COMP_ADC
    FAULT_COMP_GPIO = REG_FAULT_COMP_GPIO,
    FAULT_COMP_VCCB1 = REG_FAULT_COMP_VCCB1,
    FAULT_COMP_VCCB2 = REG_FAULT_COMP_VCCB2,
    FAULT_COMP_VCOW1 = REG_FAULT_COMP_VCOW1,
    FAULT_COMP_VCOW2 = REG_FAULT_COMP_VCOW2,
    FAULT_COMP_CBOW1 = REG_FAULT_COMP_CBOW1,
    FAULT_COMP_CBOW2 = REG_FAULT_COMP_CBOW2,
    FAULT_COMP_CBFET1 = REG_FAULT_COMP_CBFET1,
    FAULT_COMP_CBFET2 = REG_FAULT_COMP_CBFET2,
    FAULT_COMP_MISC = REG_FAULT_COMP_MISC,

    // Fault present in lower register of FAULT_OTUT
    FAULT_OT = REG_FAULT_OT,
    FAULT_UT = REG_FAULT_UT,

    // Fault present in lower register of FAULT_OVUV
    FAULT_OV1 = REG_FAULT_OV1,
    FAULT_OV2 = REG_FAULT_OV2,
    FAULT_UV1 = REG_FAULT_UV1,
    FAULT_UV2 = REG_FAULT_UV2,

    // Fault present in lower register of FAULT_PWR
    FAULT_PWR_1 = REG_FAULT_PWR1,
    FAULT_PWR_2 = REG_FAULT_PWR2,
    FAULT_PWR_3 = REG_FAULT_PWR3,
} BQ75614_FaultType;

/**
 * @brief Enums of the different ping commands that can be sent to the BQ75614
 */
typedef enum BQ75614_PingType
{
    PING_SLEEP_TO_ACTIVE = 250, // Minimum time in us for RX pin to be low to go from sleep to active mode
    PING_WAKE = 2000,           // Minimum time in us for RX pin to be low to wake up the device
    PING_SHUTDOWN = 7000,       // Minimum time in us for RX pin to be low to go from sleep to active mode
    PING_HW_RESET = 36000,      // Minimum time in us for RX pin to be low to hard reset the device
} BQ75614_PingType;

/**
 * @brief Enum of the different frame types for UART communication
 */
typedef enum BQ75614_FrameType
{
    RESPONSE_FRAME = 0,
    COMMAND_FRAME = 1,
} BQ75614_FrameType;

/**
 * @brief Enum of the different request types in frames for UART communication
 */
typedef enum BQ75614_REQ_TYPEType
{
    SINGLE_DEVICE_READ = 0,
    SINGLE_DEVICE_WRITE = 1,
    BROADCAST_READ = 4,
    BROADCAST_WRITE = 5,
} BQ75614_REQ_TYPEType;

/**
 * @brief Enum of available thresholds for the die temperature warning
 */
typedef enum BQ75614_DieWarningThresholdType
{
    BQ75614_DIE_TWARN_85 = 0,
    BQ75614_DIE_TWARN_95 = 1,
    BQ75614_DIE_TWARN_105 = 2, // Default value
    BQ75614_DIE_TWARN_115 = 3,
} BQ75614_DieWarningThresholdType;

/**
 * @brief Enum of available duration the BQ75614 will sleep before going to shutdown
 */
typedef enum BQ75614_SleepTimeType
{
    BQ75614_SLP_TIME_NONE = 0,
    BQ75614_SLP_TIME_5S = 1,
    BQ75614_SLP_TIME_10S = 2,
    BQ75614_SLP_TIME_1MIN = 3,
    BQ75614_SLP_TIME_10MIN = 4,
    BQ75614_SLP_TIME_30MIN = 5,
    BQ75614_SLP_TIME_1H = 6,
    BQ75614_SLP_TIME_2H = 7,
} BQ75614_SleepTimeType;

/**
 * @brief Enum of available duty cycles for the cell balancing
 * @note The duty cycle period will define the cycle time between balancing odd and even cells,
 */
typedef enum BQ75614_CellBalancingDutyCycleType
{
    BQ75614_CELL_DUTY_5S = 0x0,  // 5 seconds, Default value
    BQ75614_CELL_DUTY_10S = 0x1, // 10 seconds
    BQ75614_CELL_DUTY_30S = 0x2, // 30 seconds
    BQ75614_CELL_DUTY_60S = 0x3, // 60 seconds
    BQ75614_CELL_DUTY_5M = 0x4,  // 5 minutes
    BQ75614_CELL_DUTY_10M = 0x5, // 10 minutes
    BQ75614_CELL_DUTY_20M = 0x6, // 20 minutes
    BQ75614_CELL_DUTY_30M = 0x7, // 30 minutes
} BQ75614_CellBalancingDutyCycleType;

/**
 * @brief Enum of available durations during which cell balancing will be active
 */
typedef enum BQ75614_CellBalancingTimerType
{
    BQ75614_CELL_TIMER_0S = 0x0,    // 0 seconds = stop balancing
    BQ75614_CELL_TIMER_10S = 0x1,   // 10 seconds
    BQ75614_CELL_TIMER_30S = 0x2,   // 30 seconds
    BQ75614_CELL_TIMER_60S = 0x3,   // 60 seconds
    BQ75614_CELL_TIMER_5M = 0x4,    // 5 minutes = 300 seconds
    BQ75614_CELL_TIMER_10M = 0x5,   // 10 minutes
    BQ75614_CELL_TIMER_20M = 0x6,   // 20 minutes
    BQ75614_CELL_TIMER_30M = 0x7,   // 30 minutes
    BQ75614_CELL_TIMER_40M = 0x8,   // 40 minutes
    BQ75614_CELL_TIMER_50M = 0x9,   // 50 minutes
    BQ75614_CELL_TIMER_60M = 0xA,   // 60 minutes
    BQ75614_CELL_TIMER_70M = 0xB,   // 70 minutes
    BQ75614_CELL_TIMER_80M = 0xC,   // 80 minutes
    BQ75614_CELL_TIMER_90M = 0xD,   // 90 minutes
    BQ75614_CELL_TIMER_100M = 0xE,  // 100 minutes
    BQ75614_CELL_TIMER_110M = 0xF,  // 110 minutes
    BQ75614_CELL_TIMER_120M = 0x10, // 120 minutes
    BQ75614_CELL_TIMER_150M = 0x11, // 150 minutes
    BQ75614_CELL_TIMER_180M = 0x12, // 180 minutes
    BQ75614_CELL_TIMER_210M = 0x13, // 210 minutes
    BQ75614_CELL_TIMER_240M = 0x14, // 240 minutes
    BQ75614_CELL_TIMER_270M = 0x15, // 270 minutes
    BQ75614_CELL_TIMER_300M = 0x16, // 300 minutes
    BQ75614_CELL_TIMER_330M = 0x17, // 330 minutes
    BQ75614_CELL_TIMER_360M = 0x18, // 360 minutes
    BQ75614_CELL_TIMER_390M = 0x19, // 390 minutes
    BQ75614_CELL_TIMER_420M = 0x1A, // 420 minutes
    BQ75614_CELL_TIMER_450M = 0x1B, // 450 minutes
    BQ75614_CELL_TIMER_480M = 0x1C, // 480 minutes
    BQ75614_CELL_TIMER_510M = 0x1D, // 510 minutes
    BQ75614_CELL_TIMER_540M = 0x1E, // 540 minutes
    BQ75614_CELL_TIMER_600M = 0x1F, // 600 minutes
} BQ75614_CellBalancingTimerType;

/**
 * @brief Enum of available modes for the cell balancing
 * @note See chapter "8.3.3 Cell Balancing" in the datasheet for more details
 */
typedef enum BQ75614_CellBalancingModeType
{
    BQ75614_BALANCING_MODE_MANUAL = 0, // The user must choose which cells to balance and must control everything
    BQ75614_BALANCING_MODE_AUTO = 1,   // The device will cycle between odd and even cells automatically and stops after some conditions
} BQ75614_CellBalancingModeType;

/**
 * @brief Enum of available cutoff frequencies for the low pass filter for MAIN and AUX ADCs
 */
typedef enum BQ75614_LPFType
{
    BQ75614_LPF_6_5HZ = 0, // 6.5 Hz, 154 ms average
    BQ75614_LPF_13HZ = 1,  // 13 Hz, 77 ms average
    BQ75614_LPF_26HZ = 2,  // 26 Hz, 38 ms average
    BQ75614_LPF_53HZ = 3,  // 53 Hz, 19 ms average
    BQ75614_LPF_111HZ = 4, // 111 Hz, 9 ms average
    BQ75614_LPF_240HZ = 5, // 240 Hz, 4 ms average
    BQ75614_LPF_600HZ = 6, // 600 Hz, 1.6 ms average
} BQ75614_LPFType;

/**
 * @brief Enum of available modes for the main ADC running
 */
typedef enum BQ75614_MainADCModeType
{
    BQ75614_MAIN_ADC_MODE_STOP = 0,           // Main ADC is stopped
    BQ75614_MAIN_ADC_MODE_8RR = 1,            // Main ADC runs for 8 round robin cycles
    BQ75614_MAIN_ADC_MODE_CONTINUOUS_RUN = 2, // Main ADC runs continuously
} BQ75614_MainADCModeType;

/**
 * @brief TODO : Add description
 */
typedef enum BQ75614_DecimationRatioType
{
    BQ75614_DECIMATIONRATIO_0_768MS = 0,  // 0.768 ms
    BQ75614_DECIMATIONRATIO_1_536MS = 1,  // 1.536 ms
    BQ75614_DECIMATIONRATIO_3_072MS = 2,  // 3.072 ms
    BQ75614_DECIMATIONRATIO_12_288MS = 3, // 12.288 ms
} BQ75614_DecimationRatioType;

/**
 * @brief Enum of available communication timeout
 */
typedef enum BQ75614_CommTimeoutType
{
    BQ75614_TIMEOUT_DISABLE = 0, // Disable the communication timeout
    BQ75614_TIMEOUT_100MS = 1,   // 100 ms
    BQ75614_TIMEOUT_2S = 2,      // 2 s
    BQ75614_TIMEOUT_10S = 3,     // 10 s
    BQ75614_TIMEOUT_1M = 4,      // 1 min
    BQ75614_TIMEOUT_10M = 5,     // 10 min
    BQ75614_TIMEOUT_30M = 6,     // 30 min
    BQ75614_TIMEOUT_1H = 7,      // 1 hour
} BQ75614_CommTimeoutType;

/**
 * @brief Struct to contain the fault status of the BQ75614
 */
typedef struct BQ75614_FaultStruct
{
    uint8_t fault_summary;
    uint8_t fault_comm1;
    uint8_t fault_otp;
    uint8_t fault_sys;
    uint8_t fault_prot1;
    uint8_t fault_prot2;
    uint8_t fault_ov1;
    uint8_t fault_ov2;
    uint8_t fault_uv1;
    uint8_t fault_uv2;
    uint8_t fault_ot;
    uint8_t fault_ut;
    uint8_t fault_comp_gpio;
    uint8_t fault_comp_vccb1;
    uint8_t fault_comp_vccb2;
    uint8_t fault_comp_vcow1;
    uint8_t fault_comp_vcow2;
    uint8_t fault_comp_cbow1;
    uint8_t fault_comp_cbow2;
    uint8_t fault_comp_cbfet1;
    uint8_t fault_comp_cbfet2;
    uint8_t fault_comp_misc;
    uint8_t fault_pwr1;
    uint8_t fault_pwr2;
    uint8_t fault_pwr3;
} BQ75614_FaultStructType;

// Default configuration of the BQ75614
#define BQ75614_DEFAULT_CONFIG {.device_address = 1,                                                 \
                                .active_cells = 14,                                                  \
                                .overvoltage_threshold = 4200,                                       \
                                .undervoltage_threshold = 2900,                                      \
                                .short_comm_timeout = BQ75614_TIMEOUT_DISABLE,                       \
                                .long_comm_timeout_action_to_do = 0,                                 \
                                .long_comm_timeout = BQ75614_TIMEOUT_DISABLE,                        \
                                .sleep_time = BQ75614_SLP_TIME_NONE,                                 \
                                .tx_holdoff = 0,                                                     \
                                .die_warning_threshold = BQ75614_DIE_TWARN_85,                       \
                                .current_sense_enable = 0,                                           \
                                .decimation_ratio = BQ75614_DECIMATIONRATIO_0_768MS,                 \
                                .current_sense_lpf = BQ75614_LPF_6_5HZ,                              \
                                .current_sense_lpf_enable = 0,                                       \
                                .cell_voltage_lpf = BQ75614_LPF_6_5HZ,                               \
                                .cell_voltage_lpf_enable = 0,                                        \
                                .main_adc_mode = BQ75614_MAIN_ADC_MODE_CONTINUOUS_RUN,               \
                                .main_adc_delay = 0,                                                 \
                                .main_adc_enable = 1,                                                \
                                .OVUV_enable = 1,                                                    \
                                .OVUV_disabled_undervoltage_cells = BQ75614_CELL16 | BQ75614_CELL15, \
                                .OVUV_overvoltage_threshold = 4200,                                  \
                                .OVUV_undervoltage_threshold = 2900,                                 \
                                .OTUT_enable = 1,                                                    \
                                .OTUT_gpios_used = 0x01,                                             \
                                .OTUT_overtemperature_threshold = 39,                                \
                                .OTUT_undertemperature_threshold = 80,                               \
                                .cell_balancing_enable = 1,                                          \
                                .cell_balancing_mode = BQ75614_BALANCING_MODE_AUTO,                  \
                                .cell_balancing_duty_cycle = BQ75614_CELL_DUTY_30S,                  \
                                .cell_balancing_timer = BQ75614_CELL_TIMER_600M,                     \
                                .cell_balancing_overtemperature_threshold = 24,                      \
                                .cell_balancing_cool_off_ratio = 4,                                  \
                                .cell_balancing_vcb_done_threshold = 3000,                           \
                                .cell_balancing_stop_at_fault_enable = 1,                            \
                                .faults_to_mask = MASK_PWR,                                          \
                                .nfault_pin_enable = 1}

typedef struct BQ75614_ConfigStruct
{
    uint8_t device_address;                                       // Default address of the BQ75614
    uint8_t active_cells;                                         // Number of active cells
    uint32_t overvoltage_threshold;                               // Overvoltage threshold
    uint32_t undervoltage_threshold;                              // Undervoltage threshold
    BQ75614_CommTimeoutType short_comm_timeout;                   // Short communication timeout before sending a notification to the host through the NFAULT pin
    uint8_t long_comm_timeout_action_to_do;                       // Action to do when long communication timeout is reached
    BQ75614_CommTimeoutType long_comm_timeout;                    // Long communication timeout before doing action selected above
    BQ75614_SleepTimeType sleep_time;                             // Time before going to shutdown when in sleep mode
    uint8_t tx_holdoff;                                           // Number of bit cycles to wait before responding to UART
    BQ75614_DieWarningThresholdType die_warning_threshold;        // Die temperature warning threshold
    uint8_t current_sense_enable;                                 // Enable the current sense ADC
    BQ75614_DecimationRatioType decimation_ratio;                 // Decimation ratio for the current sense ADC
    BQ75614_LPFType current_sense_lpf;                            // Low pass filter for the current sense ADC
    uint8_t current_sense_lpf_enable;                             // Enable the low pass filter for the current sense ADC
    BQ75614_LPFType cell_voltage_lpf;                             // Low pass filter for the cell voltage ADC
    uint8_t cell_voltage_lpf_enable;                              // Enable the low pass filter for the cell voltage ADC
    uint8_t main_adc_enable;                                      // Enable the main ADC
    BQ75614_MainADCModeType main_adc_mode;                        // Mode of the main ADC
    uint8_t main_adc_delay;                                       // Delay before the main ADC starts
    uint8_t OVUV_enable;                                          // Enable the OVUV protection
    uint16_t OVUV_disabled_undervoltage_cells;                    // Cells who needs undervoltage protection to be disabled
    uint16_t OVUV_overvoltage_threshold;                          // Overvoltage threshold for OVUV protection
    uint16_t OVUV_undervoltage_threshold;                         // Undervoltage threshold for OVUV protection
    uint8_t OTUT_enable;                                          // Enable the OTUT protection
    uint8_t OTUT_gpios_used;                                      // GPIOs used for the OTUT protection, on which we connect a NTC
    uint8_t OTUT_overtemperature_threshold;                       // Overtemperature threshold for OTUT protection
    uint8_t OTUT_undertemperature_threshold;                      // Undertemperature threshold for OTUT protection
    uint8_t cell_balancing_enable;                                // Enable the cell balancing
    BQ75614_CellBalancingModeType cell_balancing_mode;            // Mode of the cell balancing
    BQ75614_CellBalancingDutyCycleType cell_balancing_duty_cycle; // Duty cycle of the cell balancing (switching between odd and even cells, auto mode only)
    BQ75614_CellBalancingTimerType cell_balancing_timer;          // Duration of the cell balancing
    uint8_t cell_balancing_overtemperature_threshold;             // Overtemperature threshold while cell balancing
    uint8_t cell_balancing_cool_off_ratio;                        // Cool off ratio for cell balancing
    uint16_t cell_balancing_vcb_done_threshold;                   // Voltage threshold for cell balancing to be done
    uint8_t cell_balancing_stop_at_fault_enable;                  // If balancing stops when a fault is detected
    BQ75614_MaskFaultType faults_to_mask;                         // Faults to mask
    uint8_t nfault_pin_enable;                                    // Enable the NFAULT pin
} BQ75614_ConfigStruct;

// External platform dependent functions that user MUST implement

/**
 * @brief This function must be implemented by the user to read data from the UART
 *
 * @param data The buffer where the data will be stored
 * @param size Number of bytes to read. The buffer must be at least this size
 * @return BQ75614_StatusType BQ75614_OK if the read was successful
 */
extern BQ75614_StatusType BQ75614_ReadUART(uint8_t *data, uint8_t size);

/**
 * @brief This function must be implemented by the user to write data to the UART
 *
 * @param data The buffer containing the data to write
 * @param size Number of bytes to write
 * @return BQ75614_StatusType BQ75614_OK if the write was successful
 */
extern BQ75614_StatusType BQ75614_WriteUART(uint8_t *data, uint8_t size);

/**
 * @brief This function must be implemented by the user to set RX pin low for a specific time.
 *
 * @param bq75614 The BQ75614 handle
 * @param time The minimum time in `us` to keep the RX pin low
 * @return BQ75614_StatusType BQ75614_OK if the RX pin was set low for the specified time
 */
extern BQ75614_StatusType BQ75614_PingSetLowTime(BQ75614_HandleTypeDef *bq75614, int time);

/**
 * @brief This function must be implemented by the user to provide a library delay function.
 *
 * @param ms The time in milliseconds to delay
 * @return BQ75614_StatusType BQ75614_OK if the delay was successful
 */
extern BQ75614_StatusType BQ75614_Delay(uint32_t ms);

// Init function

BQ75614_StatusType BQ75614_Init(BQ75614_HandleTypeDef *bq75614, BQ75614_ConfigStruct *bq75614_config);

// state functions

BQ75614_StatusType BQ75614_Sleep(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_Shutdown(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_SoftReset(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_SetShortCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType timeout);
BQ75614_StatusType BQ75614_GetShortCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType *timeout);
BQ75614_StatusType BQ75614_SetLongCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType timeout, uint8_t action_to_do);
BQ75614_StatusType BQ75614_GetLongCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType *timeout, uint8_t *action_to_do);
BQ75614_StatusType BQ75614_GetSleepTime(BQ75614_HandleTypeDef *bq75614, BQ75614_SleepTimeType *time);
BQ75614_StatusType BQ75614_SetSleepTime(BQ75614_HandleTypeDef *bq75614, BQ75614_SleepTimeType time);
BQ75614_StatusType BQ75614_SetTxHoldOFF(BQ75614_HandleTypeDef *bq75614, uint8_t cycles_to_wait);
BQ75614_StatusType BQ75614_GetTxHoldOFF(BQ75614_HandleTypeDef *bq75614, uint8_t *cycles_to_wait);

// Misc functions

BQ75614_StatusType BQ75614_GetPARTID(BQ75614_HandleTypeDef *bq75614, uint8_t *part_id);
BQ75614_StatusType BQ75614_GetDEVREVID(BQ75614_HandleTypeDef *bq75614, uint8_t *dev_revid);
BQ75614_StatusType BQ75614_GetDIEID(BQ75614_HandleTypeDef *bq75614, uint8_t *die_id);
BQ75614_StatusType BQ75614_SetActiveCells(BQ75614_HandleTypeDef *bq75614, uint8_t nbr_of_cells);
BQ75614_StatusType BQ75614_GetActiveCells(BQ75614_HandleTypeDef *bq75614, uint8_t *nbr_of_cells);
BQ75614_StatusType BQ75614_ReadCellVoltage(BQ75614_HandleTypeDef *bq75614, uint8_t cell, float *voltage);
BQ75614_StatusType BQ75614_ReadCellsVoltage(BQ75614_HandleTypeDef *bq75614, float *voltage);
BQ75614_StatusType BQ75614_ReadPackVoltage(BQ75614_HandleTypeDef *bq75614, float *voltage);
BQ75614_StatusType BQ75614_GetDie1Temperature(BQ75614_HandleTypeDef *bq75614, float *temperature);
BQ75614_StatusType BQ75614_GetDie2Temperature(BQ75614_HandleTypeDef *bq75614, float *temperature);
BQ75614_StatusType BQ75614_GetDieWarningThreshold(BQ75614_HandleTypeDef *bq75614, BQ75614_DieWarningThresholdType *threshold);
BQ75614_StatusType BQ75614_SetDieWarningThreshold(BQ75614_HandleTypeDef *bq75614, BQ75614_DieWarningThresholdType threshold);

// current functions

BQ75614_StatusType BQ75614_EnableCurrentSense(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_GetCurrentSense(BQ75614_HandleTypeDef *bq75614, float *current);
BQ75614_StatusType BQ75614_GetMainADCCurrentSense(BQ75614_HandleTypeDef *bq75614, float *current);
BQ75614_StatusType BQ75614_SetDecimationRatio(BQ75614_HandleTypeDef *bq75614, BQ75614_DecimationRatioType ratio);
BQ75614_StatusType BQ75614_GetDecimationRatio(BQ75614_HandleTypeDef *bq75614, BQ75614_DecimationRatioType *ratio);

// MAIN ADC functions

BQ75614_StatusType BQ75614_ConfigMainADC(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType mode, uint8_t delay);
BQ75614_StatusType BQ75614_EnableMainADC(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableMainADC(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsMainADCEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_SetMainADCMode(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType mode);
BQ75614_StatusType BQ75614_GetMainADCMode(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType *mode);
BQ75614_StatusType BQ75614_SetMainADCDelay(BQ75614_HandleTypeDef *bq75614, uint8_t delay);
BQ75614_StatusType BQ75614_GetMainADCDelay(BQ75614_HandleTypeDef *bq75614, uint8_t *delay);
BQ75614_StatusType BQ75614_EnableSRLowPassFilter(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableSRLowPassFilter(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsSRLowPassFilterEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_SetSRLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType filter);
BQ75614_StatusType BQ75614_GetSRLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType *filter);
BQ75614_StatusType BQ75614_EnableVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsVCELLLowPassFilterEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_SetVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType filter);
BQ75614_StatusType BQ75614_GetVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType *filter);

// AUX ADC functions

// TODO: Add functions for AUX ADC

// Protecton functions

// Voltage protection

BQ75614_StatusType BQ75614_ConfigOVUV(BQ75614_HandleTypeDef *bq75614, uint32_t over_voltage, uint32_t under_voltage);
BQ75614_StatusType BQ75614_SetOverVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t voltage);
BQ75614_StatusType BQ75614_GetOverVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t *voltage);
BQ75614_StatusType BQ75614_SetUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t voltage);
BQ75614_StatusType BQ75614_GetUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t *voltage);
BQ75614_StatusType BQ75614_IsOVUVEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *state);
BQ75614_StatusType BQ75614_StartOVUV(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableUnderVoltage(BQ75614_HandleTypeDef *bq75614, BQ75614_Cell uv_cells_to_disable);
BQ75614_StatusType BQ75614_EnableUnderVoltage(BQ75614_HandleTypeDef *bq75614, BQ75614_Cell uv_cells_to_enable);
BQ75614_StatusType BQ75614_GetDisabledUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint16_t *uv_disabled);

// Temperature protection

BQ75614_StatusType BQ75614_ConfigOTUT(BQ75614_HandleTypeDef *bq75614, uint8_t over_temperature_ratio, uint8_t under_temperature_ratio, uint8_t gpios);
BQ75614_StatusType BQ75614_EnableTSREF(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableTSREF(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsTSREFEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_GetTSREF(BQ75614_HandleTypeDef *bq75614, float *voltage);
BQ75614_StatusType BQ75614_SetOverTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t ratio);
BQ75614_StatusType BQ75614_GetOverTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t *ratio);
BQ75614_StatusType BQ75614_SetUnderTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t ratio);
BQ75614_StatusType BQ75614_GetUnderTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t *ratio);
BQ75614_StatusType BQ75614_StartOTUT(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsOTUTEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
// BQ75614_StatusType BQ75614_SetWarningLevels(BQ75614_HandleTypeDef *bq75614, float over_voltage, float under_voltage, float over_current, float over_temperature);

// GPIO functions

BQ75614_StatusType BQ75614_GetGPIOConfig(BQ75614_HandleTypeDef *bq75614, BQ75614_GPIO gpio, BQ75614_GPIOConf *config); // Be aware of special config chap 8.3.5
BQ75614_StatusType BQ75614_SetGPIOConfig(BQ75614_HandleTypeDef *bq75614, BQ75614_GPIO gpio, BQ75614_GPIOConf config);  // Be aware of special config chap 8.3.5
BQ75614_StatusType BQ75614_SetGPIOsConfig(BQ75614_HandleTypeDef *bq75614, uint8_t gpios, BQ75614_GPIOConf config);     // Be aware of special config chap 8.3.5

// Cell balancing functions

BQ75614_StatusType BQ75614_ConfigBalancing(BQ75614_HandleTypeDef *bq75614,
                                           BQ75614_CellBalancingTimerType timer,
                                           BQ75614_CellBalancingModeType auto_mode,
                                           BQ75614_CellBalancingDutyCycleType duty_cycle,
                                           uint8_t OTCB_threshold,
                                           uint8_t cooloff,
                                           uint16_t vcb_done_threshold,
                                           uint8_t stop_at_fault);
BQ75614_StatusType BQ75614_SetVCBDoneThreshold(BQ75614_HandleTypeDef *bq75614, uint16_t threshold);
BQ75614_StatusType BQ75614_GetVCBDoneThreshold(BQ75614_HandleTypeDef *bq75614, uint16_t *threshold);
BQ75614_StatusType BQ75614_StartBalancing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableBalancing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_PauseBalancing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_GetCellBalancingRemainingTime(BQ75614_HandleTypeDef *bq75614, uint8_t cell, uint32_t *time);
BQ75614_StatusType BQ75614_GetCellsBalancingRemainingTime(BQ75614_HandleTypeDef *bq75614, uint32_t *time);
BQ75614_StatusType BQ75614_IsBalancingEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_SetBalancingDutyCycle(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingDutyCycleType duty_cycle);
BQ75614_StatusType BQ75614_GetBalancingDutyCycle(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingDutyCycleType *duty_cycle);
BQ75614_StatusType BQ75614_SetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t cell, BQ75614_CellBalancingTimerType timer);
BQ75614_StatusType BQ75614_GetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t cell, BQ75614_CellBalancingTimerType *timer);
BQ75614_StatusType BQ75614_SetCellsBalancingTimer(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingTimerType timer);
BQ75614_StatusType BQ75614_GetCellsBalancingTimer(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingTimerType *timer);
BQ75614_StatusType BQ75614_GetOTCBThreshold(BQ75614_HandleTypeDef *bq75614, uint8_t *threshold);
BQ75614_StatusType BQ75614_SetOTCBThreshold(BQ75614_HandleTypeDef *bq75614, uint8_t threshold);
BQ75614_StatusType BQ75614_GetOTCBCoolOff(BQ75614_HandleTypeDef *bq75614, uint8_t *cool_off);
BQ75614_StatusType BQ75614_SetOTCBCoolOff(BQ75614_HandleTypeDef *bq75614, uint8_t cool_off);
BQ75614_StatusType BQ75614_SetBalancingMode(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingModeType mode);
BQ75614_StatusType BQ75614_GetBalancingMode(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingModeType *mode);
BQ75614_StatusType BQ75614_EnableOTCB(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_IsOTCBEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled);
BQ75614_StatusType BQ75614_IsBalancingConfInvalid(BQ75614_HandleTypeDef *bq75614, uint8_t *invalid);
BQ75614_StatusType BQ75614_IsBalancingPaused(BQ75614_HandleTypeDef *bq75614, uint8_t *paused);

// Fault functions

BQ75614_StatusType BQ75614_GetFaultStatus(BQ75614_HandleTypeDef *bq75614, uint8_t *status);
BQ75614_StatusType BQ75614_GetAllFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_FaultStructType *status);
BQ75614_StatusType BQ75614_GetFaultLowerLevelStatus(BQ75614_HandleTypeDef *bq75614, BQ75614_FaultType fault_to_check, uint8_t *status);
BQ75614_StatusType BQ75614_ClearAllFaults(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_ClearFault(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType fault_to_clear);
BQ75614_StatusType BQ75614_MaskFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType fault_to_mask);
BQ75614_StatusType BQ75614_UnmaskFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType fault_to_unmask);
BQ75614_StatusType BQ75614_GetMaskedFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType *faults_masked);
BQ75614_StatusType BQ75614_EnableFaultPin(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_DisableFaultPin(BQ75614_HandleTypeDef *bq75614);

// UART functions

BQ75614_StatusType BQ75614_SingleDeviceRead(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size);
BQ75614_StatusType BQ75614_SingleDeviceWrite(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size);
BQ75614_StatusType BQ75614_BroadcastWrite(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size);
BQ75614_StatusType BQ75614_BroadcastRead(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size);
BQ75614_StatusType BQ75614_Ping(BQ75614_HandleTypeDef *bq75614, BQ75614_PingType ping);
BQ75614_StatusType BQ75614_WakeUpPing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_SleepToActivePing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_ShutdownPing(BQ75614_HandleTypeDef *bq75614);
BQ75614_StatusType BQ75614_HardResetPing(BQ75614_HandleTypeDef *bq75614);

// OTP functions

BQ75614_StatusType BQ75614_ProgramOTP(BQ75614_HandleTypeDef *bq75614);

// Diagnostics functions

BQ75614_StatusType BQ75614_StartPowerBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614); // not implemented yet
BQ75614_StatusType BQ75614_StartOVUVBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614);  // not implemented yet
BQ75614_StatusType BQ75614_StartOTUTBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614);  // not implemented yet
// There is a lot more... see from chapter 8.3.6.4.6 and go on

// SPI functions as master will not be implemented for now
// OTP functions will not be implemented for now

#endif // _BQ75614_H
