/**
 * @details This file contains the implementation of the BQ75614 library. It allows to communicate, configure and control the BQ75614 chip through UART.
 */
#include "bq75614.h"

#define CHECK_NULL(...) do { \
    void *arr[] = {__VA_ARGS__}; \
    for (uint64_t i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) { \
        if (arr[i] == NULL) { \
            return BQ75614_ERROR_NULL_POINTER; \
        } \
    } \
} while (0)

#define CHECK_BQ75614_ERROR(fn_call) do { \
    BQ75614_StatusType err = (fn_call); \
    if (err != BQ75614_OK) { \
        return err; \
    } \
} while (0)

// CRC functions

// CRC16 Lookup Table, This table allows to avoid the computation of the CRC polynomial
// ITU_T polynomial: x^16 + x^15 + x^2 + 1
#define CRC16_INIT 0xFFFF
const uint16_t crc16_table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301,
                                   0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
                                   0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81,
                                   0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                                   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00,
                                   0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
                                   0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380,
                                   0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141,
                                   0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
                                   0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0,
                                   0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881,
                                   0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401,
                                   0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1,
                                   0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180,
                                   0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
                                   0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01,
                                   0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1,
                                   0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80,
                                   0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                                   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200,
                                   0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1,
                                   0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780,
                                   0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41,
                                   0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901,
                                   0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
                                   0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80,
                                   0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                                   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

/**
 * @brief Compute remainder of frame for CRC16
 *
 * @param data The data from which to compute the CRC, without the CRC padding bytes
 * @param length The length of the data, without the CRC padding bytes
 * @param remainder The remainder of the CRC
 * @return BQ75614_StatusType This function will always return BQ75614_OK
 */
static BQ75614_StatusType CRC16_ComputeRemainder(uint8_t *data, int length, uint8_t *remainder)
{
    CHECK_NULL(data,remainder);

    uint16_t crc = CRC16_INIT;
    // Magic voodoo to compute the CRC, it works, don't touch !
    for (int i = 0; i < length; i++)
    {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }
    remainder[0] = crc & 0xFF;
    remainder[1] = (crc >> 8) & 0xFF;

    return BQ75614_OK;
}

/**
 * @brief Check that the CRC of the frame is correct
 *
 * @note The whole frame goes under the CRC, including the CRC bytes. The final result must be 0x0000
 *
 * @param frame The frame to check, including the CRC bytes
 * @param frame_size The size of the frame, including the CRC bytes
 * @return BQ75614_StatusType BQ75614_OK if the CRC is correct, BQ75614_ERROR_BAD_CRC otherwise
 */
static BQ75614_StatusType BQ75614_CheckCRC(uint8_t *frame, uint16_t frame_size)
{
    CHECK_NULL(frame);

    uint16_t remainder; // CRC remainder, as uint16_t and not uint8_t* because in this case we don't care about endianness
    CHECK_BQ75614_ERROR(CRC16_ComputeRemainder(frame, frame_size, (uint8_t *)&remainder));

    // Remainder must be 0x0000 to be a valid CRC
    return remainder == 0 ? BQ75614_OK : BQ75614_ERROR_BAD_CRC;
}

/**
 * @brief This function sends a frame to the BQ75614.
 * Depending, on the request type it will build a frame with the correct headers and CRC.
 * This is a helper function for `BQ75614_SingleDeviceWrite` and `BQ75614_BroadcastWrite`
 *
 * @param bq75614 The BQ75614 handle
 * @param frame_type The type of the frame to send
 * @param reg The register to write to
 * @param data The data to write, or the buffer to store the read data
 * @param data_buf_size The size of the data buffer
 * @return BQ75614_StatusType BQ75614_OK if the frame was sent successfully
 */
static BQ75614_StatusType BQ75614_SendFrame(BQ75614_HandleTypeDef *bq75614, BQ75614_REQ_TYPEType frame_type, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    CHECK_NULL(bq75614, data_buf);

    if (data_buf_size > BQ75614_MAX_WRITE_PAYLOAD_SIZE)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    if (frame_type > BROADCAST_WRITE)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    uint32_t broadcast_offset = 0; // Offset to add to the register address if we are in broadcast mode
    uint8_t frame[BQ75614_MAX_WRITE_FRAME_SIZE] = {0};
    // Init COMMAND frame with the correct request type and data size, data size is lowered by one because of spec
    frame[FRAME_INIT_BYTE_INDEX] = (COMMAND_FRAME << FRAME_TYPE_POS) | (frame_type << FRAME_REQ_TYPE_POS) | ((data_buf_size - 1) & 0x07);
    // Device address to whom we want to read
    if (frame_type == BROADCAST_WRITE)
    {
        // If it's a broadcast write, we don't need to specify the device address and all other register addresses are shifted by one in buffer
        broadcast_offset = 1;
    }
    else
    {
        // Otherwise we set the address of the device
        frame[FRAME_DEVICE_ADDRESS_INDEX] = (bq75614->device_address & 0x3F);
    }
    frame[FRAME_REG_ADDRESS_INDEX_1 - broadcast_offset] = (reg >> 8) & 0xFF;
    frame[FRAME_REG_ADDRESS_INDEX_2 - broadcast_offset] = reg & 0xFF;
    // Data to write
    memcpy(&frame[FRAME_DATA_BASE_INDEX - broadcast_offset], data_buf, data_buf_size);
    // CRC
    CRC16_ComputeRemainder(frame, BQ75614_FRAME_HEADERS_SIZE + data_buf_size - broadcast_offset, (&frame[FRAME_DATA_BASE_INDEX + data_buf_size - broadcast_offset]));

    CHECK_BQ75614_ERROR(BQ75614_WriteUART(frame, BQ75614_FRAME_HEADERS_SIZE + data_buf_size + BQ75614_FRAME_CRC_SIZE - broadcast_offset));

    return BQ75614_OK;
}

/**
 * @brief This function receives a frame from the BQ75614.
 * Depending, on the request type it will check for a frame with the correct headers and CRC.
 * This is a helper function for `BQ75614_SingleDeviceRead` and `BQ75614_BroadcastRead`
 *
 * @param bq75614 The BQ75614 handle
 * @param frame_type The type of the frame to receive
 * @param reg The register from who we expect a response
 * @param data The buffer to store the read data
 * @param data_buf_size The size of the data buffer
 * @return BQ75614_StatusType BQ75614_OK if the frame was received successfully
 */
static BQ75614_StatusType BQ75614_ReceiveFrame(BQ75614_HandleTypeDef *bq75614, BQ75614_REQ_TYPEType frame_type, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    // Frame initialization byte
    // B[7]       FRAME_TYPE ->  1 = Command frame | FRAME_TYPE 0 = Response Frame
    // If FRAME_TYPE = 1
    //      B[6:4]   = REQ_TYPE : 000 = Single Device Read, 001 = Single Device Write
    //      B[3]     = Reserved
    //      B[2:0]   = DATASIZE, excluding device address, register adress and CRC

    // Device adress byte, non existant if broadcast
    // B[7:6] = Reserved, should always write 0
    // B[5:0] = Device address

    // Register adress 2 bytes, REG_ADR, if a write is sent to a bad address, the command is ignored.
    // If a read is done on a bad address, the response is 0x00
    // B[15:8] = Register address MSB
    // B[7:0]  = Register address LSB

    // Data bytes
    // If FRAME_TYPE = 1, command frame
    //      If REQ_TYPE = 000, Single Device Read
    //          Byte[0] = number of bytes to read
    //      If REQ_TYPE = 001, Single Device Write
    //          Byte[n:0] = Data to write to REG_ADR

    // CRC

    // Read datasheet SLUSDT5B , chap. "8.3.6.1.1.2.1 Transaction Frame Structure" for more information

    CHECK_NULL(bq75614, data_buf);

    uint32_t broadcast_offset = 0; // Offset to add to the register address if we are in broadcast mode
    uint8_t frame[BQ75614_MAX_READ_FRAME_SIZE] = {0};
    // Init COMMAND frame of type given in paramater. DATA_SIZE must be 0. (from datasheet)
    frame[FRAME_INIT_BYTE_INDEX] = (COMMAND_FRAME << FRAME_TYPE_POS) | (frame_type << FRAME_REQ_TYPE_POS);
    if (frame_type == BROADCAST_READ)
    {
        broadcast_offset = 1;
    }
    else
    {
        // Device address to whom we want to read
        frame[FRAME_DEVICE_ADDRESS_INDEX] = (bq75614->device_address & 0x3F);
    }
    frame[FRAME_REG_ADDRESS_INDEX_1 - broadcast_offset] = (reg >> 8) & 0xFF;
    frame[FRAME_REG_ADDRESS_INDEX_2 - broadcast_offset] = reg & 0xFF;
    // Number of bytes to read
    frame[FRAME_DATA_BASE_INDEX - broadcast_offset] = data_buf_size - 1; // Because it's written on documentation that it's the number of bytes to read - 1
    // CRC, we always send one byte of data, which is the number of bytes to read
    CRC16_ComputeRemainder(frame, BQ75614_FRAME_HEADERS_SIZE + 1 - broadcast_offset, (&frame[FRAME_DATA_BASE_INDEX + 1 - broadcast_offset]));

    CHECK_BQ75614_ERROR(BQ75614_WriteUART(frame, BQ75614_FRAME_HEADERS_SIZE + 1 + BQ75614_FRAME_CRC_SIZE - broadcast_offset));
    // Now we must wait for a response and check the CRC first
    CHECK_BQ75614_ERROR(BQ75614_ReadUART(frame, data_buf_size + BQ75614_FRAME_HEADERS_SIZE + BQ75614_FRAME_CRC_SIZE));
    CHECK_BQ75614_ERROR(BQ75614_CheckCRC(frame, data_buf_size + BQ75614_FRAME_HEADERS_SIZE + BQ75614_FRAME_CRC_SIZE));
    
    // Test that we received a response frame from the correct device at the correct register
    if ((((frame[FRAME_INIT_BYTE_INDEX] & (1 << FRAME_TYPE_POS)) != RESPONSE_FRAME)) ||
        (frame[FRAME_DEVICE_ADDRESS_INDEX] != (bq75614->device_address & 0x3F)) ||
        (frame[FRAME_REG_ADDRESS_INDEX_1] != ((reg >> 8) & 0xFF)) ||
        (frame[FRAME_REG_ADDRESS_INDEX_2] != (reg & 0xFF)))
    {
        return BQ75614_ERROR_BAD_DEVICE;
    }
    // If so, we store the data in the data array
    for (int i = 0; i < data_buf_size; i++)
    {
        data_buf[i] = frame[FRAME_DATA_BASE_INDEX + i];
    }

    return BQ75614_OK;
}

/**
 * @brief This function initializes the BQ75614 handle and the BQ75614 chip.
 * It will start communication with the device and configure it with the given configuration.
 *
 * @param bq75614 The BQ75614 handle that will be initialized
 * @param bq75614_config The configuration of the BQ75614. You can init a default config with ´BQ75614_ConfigStruct bq75614_config = BQ75614_DEFAULT_CONFIG;´
 * @return BQ75614_StatusType BQ75614_OK if the BQ75614 was initialized successfully
 */
BQ75614_StatusType BQ75614_Init(BQ75614_HandleTypeDef *bq75614, BQ75614_ConfigStruct *bq75614_config)
{
    CHECK_NULL(bq75614, bq75614_config);

    BQ75614_StatusType err = BQ75614_OK;
    uint8_t part_id = 0;

    bq75614->device_address = bq75614_config->device_address;

    CHECK_BQ75614_ERROR(BQ75614_WakeUpPing(bq75614));
    err = BQ75614_GetPARTID(bq75614, &part_id);
    // Here we don't test the answer but only if device responded.
    // If we receive a `BQ75614_ERROR_UART_RECEIVE_FAILED` we can test to communicate with address 0x00.
    // If we got a response, it means that device got the Hardware reset values and it's OTP programming didn't went well.
    // The device should be replaced if it's the case. See chap 8.3.6.4.4.3 of the datasheet (SLUSDT5B)
    if (err == BQ75614_ERROR_UART_RECEIVE_FAILED)
    {
        bq75614->device_address = 0x00; // Try with address 0x00
        CHECK_BQ75614_ERROR(BQ75614_GetPARTID(bq75614, &part_id));
        // otherwise it means that the device is not correctly programmed and should be replaced
        return BQ75614_ERROR_REPLACE_DEVICE;
    }
    if (err != BQ75614_OK)
    {
        return err;
    }

    // Check that part_id is correct and not a sample chip, development chip or something else
    //	if (part_id != BQ75614_PARTID) {
    //        return BQ75614_ERROR_BAD_PARTID;
    //	}

    // Set active number of cells
    CHECK_BQ75614_ERROR(BQ75614_SetActiveCells(bq75614, bq75614_config->active_cells));
    // Set the time before we receive a communication alert because no communication was sent to the BQ75614
    CHECK_BQ75614_ERROR(BQ75614_SetShortCommTimeout(bq75614, bq75614_config->short_comm_timeout));
    // Set the time before the BQ75614 goes to sleep or shutdown mode if no communication was received
    CHECK_BQ75614_ERROR(BQ75614_SetLongCommTimeout(bq75614, bq75614_config->long_comm_timeout, bq75614_config->long_comm_timeout_action_to_do));
    // Set the time the BQ75614 will wait before going from Sleep to Shutdown
    CHECK_BQ75614_ERROR(BQ75614_SetSleepTime(bq75614, bq75614_config->sleep_time));
    // Set the bit cycles the BQ75614 will wait before responding to the host
    CHECK_BQ75614_ERROR(BQ75614_SetTxHoldOFF(bq75614, bq75614_config->tx_holdoff));
    // Set the temperature warning threshold of the Die (the chip itself) to 85°C
    CHECK_BQ75614_ERROR(BQ75614_SetDieWarningThreshold(bq75614, bq75614_config->die_warning_threshold));

    if (bq75614_config->current_sense_enable)
    {
        if (bq75614_config->current_sense_lpf_enable)
        {
            // Set the low pass filter for the current sensing
            CHECK_BQ75614_ERROR(BQ75614_SetSRLowPassFilter(bq75614, bq75614_config->current_sense_lpf));

            err = BQ75614_EnableSRLowPassFilter(bq75614);
        }
        else
        {
            err = BQ75614_DisableSRLowPassFilter(bq75614);
        }
        if (err != BQ75614_OK)
        {
            return err;
        }

        // Set the decimation ratio for current sensing, See 8.3.2.4 CS ADC of the datasheet (SLUSDT5B)
        CHECK_BQ75614_ERROR(BQ75614_SetDecimationRatio(bq75614, bq75614_config->decimation_ratio));
        // If main ADC is enabled, current sensing is enabled too !
        if (!bq75614_config->main_adc_enable)
        {
            // Enable current sensing
            CHECK_BQ75614_ERROR(BQ75614_EnableCurrentSense(bq75614));
        }
    }

    if (bq75614_config->cell_voltage_lpf_enable)
    {
        // Set the low pass filter for the Cell Voltage sensing
        CHECK_BQ75614_ERROR(BQ75614_SetVCELLLowPassFilter(bq75614, bq75614_config->cell_voltage_lpf));

        err = BQ75614_EnableVCELLLowPassFilter(bq75614);
    }
    else
    {
        err = BQ75614_DisableVCELLLowPassFilter(bq75614);
    }
    if (err != BQ75614_OK)
    {
        return err;
    }

    if (bq75614_config->main_adc_enable)
    {
        // Configure the Main ADC
        CHECK_BQ75614_ERROR(BQ75614_ConfigMainADC(bq75614, bq75614_config->main_adc_mode, bq75614_config->main_adc_delay));
    }

    if (bq75614_config->OVUV_enable)
    {
        // Configure the cells for whom we deactivate the undervoltage protection
        CHECK_BQ75614_ERROR(BQ75614_DisableUnderVoltage(bq75614, bq75614_config->OVUV_disabled_undervoltage_cells));
        // Configure OV and UV thresholds and enable it
        CHECK_BQ75614_ERROR(BQ75614_ConfigOVUV(bq75614, bq75614_config->OVUV_overvoltage_threshold, bq75614_config->OVUV_undervoltage_threshold));
    }

    if (bq75614_config->OTUT_enable)
    {
        //  Set the overtemperature and undertemperature thresholds for each Thermistor connected to a GPIO.
        CHECK_BQ75614_ERROR(BQ75614_ConfigOTUT(bq75614, bq75614_config->OTUT_overtemperature_threshold, bq75614_config->OTUT_undertemperature_threshold, bq75614_config->OTUT_gpios_used));
    }

    if (bq75614_config->cell_balancing_enable)
    {
        // Configure the balancing
        CHECK_BQ75614_ERROR(BQ75614_ConfigBalancing(bq75614,
                                      bq75614_config->cell_balancing_timer,
                                      bq75614_config->cell_balancing_mode,
                                      bq75614_config->cell_balancing_duty_cycle,
                                      bq75614_config->cell_balancing_overtemperature_threshold,
                                      bq75614_config->cell_balancing_cool_off_ratio,
                                      bq75614_config->cell_balancing_vcb_done_threshold,
                                      bq75614_config->cell_balancing_stop_at_fault_enable));
    }

    // Set the mask for the faults we don't want to trigger the `NFAULT` pin
    if (bq75614_config->faults_to_mask)
    {
        CHECK_BQ75614_ERROR(BQ75614_MaskFaults(bq75614, bq75614_config->faults_to_mask));
    }

    if (bq75614_config->nfault_pin_enable)
    {
        // We enable the `NFAULT` pin to be triggered when a fault occurs
        err = BQ75614_EnableFaultPin(bq75614);
    }
    else
    {
        err = BQ75614_DisableFaultPin(bq75614);
    }
    if (err != BQ75614_OK)
    {
        return err;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the time the BQ75614 will wait before going from sleep to shutdown
 *
 * @param bq75614 The BQ75614 handle
 * @param time The buffer to store the sleep time
 * @return BQ75614_StatusType BQ75614_OK if the sleep time was read successfully
 *
 * @note The sleep time can be none, 1s, 5s, 10s, 1min, 10min, 30min, 1h, 2h
 */
BQ75614_StatusType BQ75614_GetSleepTime(BQ75614_HandleTypeDef *bq75614, BQ75614_SleepTimeType *time)
{
    CHECK_NULL(bq75614, time);

    // Read the bits [2:0] of register PWR_TRANSIT_CONF
    uint8_t pwr_transit_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    *time = pwr_transit_conf & 0x7;

    return BQ75614_OK;
}

/**
 * @brief This function sets the time the BQ75614 will wait before going from sleep to shutdown
 *
 * @param bq75614 The BQ75614 handle
 * @param time The sleep time to set
 * @return BQ75614_StatusType BQ75614_OK if the sleep time was set successfully
 *
 * @note The sleep time can be none, 1s, 5s, 10s, 1min, 10min, 30min, 1h, 2h
 */
BQ75614_StatusType BQ75614_SetSleepTime(BQ75614_HandleTypeDef *bq75614, BQ75614_SleepTimeType time)
{
    CHECK_NULL(bq75614);

    if (time > BQ75614_SLP_TIME_2H)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Read the bits [2:0] of register PWR_TRANSIT_CONF
    uint8_t pwr_transit_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    pwr_transit_conf &= ~(0x7);
    pwr_transit_conf |= time;

    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    // Check that value was correctly written
    BQ75614_SleepTimeType time_read;
    CHECK_BQ75614_ERROR(BQ75614_GetSleepTime(bq75614, &time_read));
    if (time_read != time)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function configures the time before the BQ75614 sends an alert to the host because no communication was received.
 * If enabled, the timer is reset each time a communication is received or sent. If the timer expires, the `FAULT_SYS[CTS]` bit is set.
 * It allows to warn the host and avoid to reach the `LongCommTimeout`.
 *
 * @param bq75614 The BQ75614 handle
 * @param timeout The timeout to set, of type `BQ75614_CommTimeoutType`, `BQ75614_TIMEOUT_DISABLE``to disable
 * @return BQ75614_StatusType BQ75614_OK if the short communication timeout was set successfully
 */
BQ75614_StatusType BQ75614_SetShortCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType timeout)
{
    CHECK_NULL(bq75614);

    if (timeout > BQ75614_TIMEOUT_1H)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Reads the COMM_TIMEOUT_CONF register
    uint8_t comm_timeout_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    // Set the timeout value
    comm_timeout_conf &= ~(0x7 << COMM_TIMEOUT_CONF_CTS_TIME_POS);
    comm_timeout_conf |= (timeout << COMM_TIMEOUT_CONF_CTS_TIME_POS);

    // Write the new value
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    // Check that value was correctly written
    BQ75614_CommTimeoutType timeout_read;
    CHECK_BQ75614_ERROR(BQ75614_GetShortCommTimeout(bq75614, &timeout_read));
    if (timeout_read != timeout)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the time before the BQ75614 sends an alert to the host because no communication was received.
 *
 * @param bq75614 The BQ75614 handle
 * @param timeout The buffer to store the short communication timeout
 * @return BQ75614_StatusType BQ75614_OK if the short communication timeout was read successfully
 */
BQ75614_StatusType BQ75614_GetShortCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType *timeout)
{
    CHECK_NULL(bq75614, timeout);

    // Reads the COMM_TIMEOUT_CONF register
    uint8_t comm_timeout_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    *timeout = (comm_timeout_conf >> COMM_TIMEOUT_CONF_CTS_TIME_POS) & 0x7;

    return BQ75614_OK;
}

/**
 * @brief This function configures the time before the BQ75614 goes to `sleep` or `shutdown` mode if no communication was received.
 * If enabled, the timer is reset each time a communication is received or sent.
 * If the timer expires, the BQ75614 goes to mode selected in `COMM_TIMEOUT_CONF[CTL_ACT]`.
 *
 * @param bq75614 The BQ75614 handle
 * @param timeout The timeout to set, of type `BQ75614_CommTimeoutType`. 0 is disabled.
 * @param action_to_do The action to do if the timeout expires.
 * 0 -> sets `FAULT_SYS[CTL]` and sends device to SLEEP mode. 1 -> sends device to SHUTDOWN mode. Don't care if timeout is disabled.
 * @return BQ75614_StatusType BQ75614_OK if the long communication timeout was set successfully
 */
BQ75614_StatusType BQ75614_SetLongCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType timeout, uint8_t action_to_do)
{
    CHECK_NULL(bq75614);

    // Test if action_to_do and timeout are in the right range
    if ((action_to_do > 1) || (timeout > BQ75614_TIMEOUT_1H))
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // Reads the COMM_TIMEOUT_CONF register
    uint8_t comm_timeout_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    // Set the timeout value
    comm_timeout_conf &= ~(0x7 << COMM_TIMEOUT_CONF_CTL_TIME_POS);
    comm_timeout_conf |= ((timeout & 0x07) << COMM_TIMEOUT_CONF_CTL_TIME_POS);
    // Set the action to do
    comm_timeout_conf &= ~(1 << COMM_TIMEOUT_CONF_CTL_ACT_POS);
    comm_timeout_conf |= ((action_to_do & 0x01) << COMM_TIMEOUT_CONF_CTL_ACT_POS);

    // Write the new value
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    // Check that value was correctly written
    BQ75614_CommTimeoutType timeout_read;
    uint8_t action_to_do_read;
    CHECK_BQ75614_ERROR(BQ75614_GetLongCommTimeout(bq75614, &timeout_read, &action_to_do_read));
    if ((timeout_read != timeout) || (action_to_do_read != action_to_do))
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the time before the BQ75614 goes to `sleep` or `shutdown` mode if no communication was received.
 *
 * @param bq75614 The BQ75614 handle
 * @param timeout The buffer to store the long communication timeout
 * @param action_to_do The buffer to store the action to do if the timeout expires.
 * 0 -> sets `FAULT_SYS[CTL]` and sends device to SLEEP mode. 1 -> sends device to SHUTDOWN mode.
 * @return BQ75614_StatusType BQ75614_OK if the long communication timeout was read successfully
 */
BQ75614_StatusType BQ75614_GetLongCommTimeout(BQ75614_HandleTypeDef *bq75614, BQ75614_CommTimeoutType *timeout, uint8_t *action_to_do)
{
    CHECK_NULL(bq75614, timeout, action_to_do);

    // Reads the COMM_TIMEOUT_CONF register
    uint8_t comm_timeout_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_COMM_TIMEOUT_CONF, &comm_timeout_conf, 1));

    *timeout = (comm_timeout_conf >> COMM_TIMEOUT_CONF_CTL_TIME_POS) & 0x7;
    *action_to_do = (comm_timeout_conf >> COMM_TIMEOUT_CONF_CTL_ACT_POS) & 0x1;

    return BQ75614_OK;
}

/**
 * @brief This function sets the number of UART bit cycles the BQ75614 will wait before responding to the host
 *
 * @param bq75614 The BQ75614 handle
 * @param cycles_to_wait The number of cycles to wait before responding to the host
 * @return BQ75614_StatusType BQ75614_OK if the number of cycles to wait was set successfully
 */
BQ75614_StatusType BQ75614_SetTxHoldOFF(BQ75614_HandleTypeDef *bq75614, uint8_t cycles_to_wait)
{
    CHECK_NULL(bq75614);

    // Write the new value
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_TX_HOLD_OFF, &cycles_to_wait, 1));

    // Check that value was correctly written
    uint8_t cycles_to_wait_read;
    CHECK_BQ75614_ERROR(BQ75614_GetTxHoldOFF(bq75614, &cycles_to_wait_read));
    if (cycles_to_wait_read != cycles_to_wait)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the number of UART bit cycles the BQ75614 will wait before responding to the host
 *
 * @param bq75614 The BQ75614 handle
 * @param cycles_to_wait The buffer to store the number of cycles to wait before responding to the host
 * @return BQ75614_StatusType BQ75614_OK if the number of cycles to wait was read successfully
 */
BQ75614_StatusType BQ75614_GetTxHoldOFF(BQ75614_HandleTypeDef *bq75614, uint8_t *cycles_to_wait)
{
    CHECK_NULL(bq75614, cycles_to_wait);

    // Reads the REG_TX_HOLD_OFF register
    uint8_t cycles_to_wait_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_TX_HOLD_OFF, &cycles_to_wait_read, 1));

    *cycles_to_wait = cycles_to_wait_read;

    return BQ75614_OK;
}

/**
 * @brief This function returns the warning temperature threshold for the DIE of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The buffer to store the warning temperature threshold
 * @return BQ75614_StatusType BQ75614_OK if the warning temperature threshold was read successfully
 *
 * @note The warning temperature are 85, 95, 105 or 115 degrees Celsius
 */
BQ75614_StatusType BQ75614_GetDieWarningThreshold(BQ75614_HandleTypeDef *bq75614, BQ75614_DieWarningThresholdType *threshold)
{
    CHECK_NULL(bq75614, threshold);

    // Read the bits [4:3] of register PWR_TRANSIT_CONF
    uint8_t pwr_transit_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    *threshold = (pwr_transit_conf >> PWR_TRANSIT_CONF_TWARN_THR_POS) & 0x3;

    return BQ75614_OK;
}

/**
 * @brief This function sets the warning temperature threshold for the DIE of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The warning temperature threshold to set
 * @return BQ75614_StatusType BQ75614_OK if the warning temperature threshold was set successfully
 *
 * @note The warning temperature are 85, 95, 105 or 115 degrees Celsius
 */
BQ75614_StatusType BQ75614_SetDieWarningThreshold(BQ75614_HandleTypeDef *bq75614, BQ75614_DieWarningThresholdType threshold)
{
    CHECK_NULL(bq75614);

    if (threshold > BQ75614_DIE_TWARN_115)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Read the bits [4:3] of register PWR_TRANSIT_CONF
    uint8_t pwr_transit_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    pwr_transit_conf &= ~(0x3 << PWR_TRANSIT_CONF_TWARN_THR_POS);
    pwr_transit_conf |= (threshold << PWR_TRANSIT_CONF_TWARN_THR_POS);

    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_PWR_TRANSIT_CONF, &pwr_transit_conf, 1));

    // Check that value was correctly written
    BQ75614_DieWarningThresholdType threshold_read;
    CHECK_BQ75614_ERROR(BQ75614_GetDieWarningThreshold(bq75614, &threshold_read));
    if (threshold_read != threshold)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This private function reads the voltage of the cells from the BQ75614.
 *
 * @param bq75614 The BQ75614 handle
 * @param starting_cell The cell number to start reading from. It should be between 1 and 16
 * @param nbr_of_cells_to_read The number of cells to read. It should be between 1 and 16
 * @param voltage The buffer to store the voltages in mVolts. The buffer must be at least of size `nbr_of_cells_to_read`.
 * voltage[0] will contain cell's voltage 1, and so on.
 * @return BQ75614_StatusType BQ75614_OK if the voltage was read successfully
 */
static BQ75614_StatusType ReadCellsVoltage(BQ75614_HandleTypeDef *bq75614, uint8_t starting_cell, uint8_t nbr_of_cells_to_read, float *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // We must first verify that Main ADC is running
    uint8_t enabled;
    CHECK_BQ75614_ERROR(BQ75614_IsMainADCEnabled(bq75614, &enabled));
    if (!enabled)
    {
        return BQ75614_ERROR_MAIN_ADC_NOT_RUNNING;
    }

    // Then we check that the required cells are in the range of the active cells
    if (starting_cell + nbr_of_cells_to_read - 1 > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We can finally read the voltage of the cell
    // Register addresses for cell's voltage start from Cell 16 and goes to cell 1
    uint16_t reg = REG_VCELL1_HI - (((starting_cell - 1) + (nbr_of_cells_to_read - 1)) * 2);
    uint8_t data[BQ75614_MAX_ACTIVE_CELLS * 2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, reg, data, nbr_of_cells_to_read * 2));

    int16_t temp_voltage;
    // convert all the data to voltage
    for (int i = 0; i < nbr_of_cells_to_read; i++)
    {
        // The voltage is stored in 2 bytes, the first byte is the MSB and the second byte is the LSB
        // We must first convert the 2 bytes into a 16 bits integer
        temp_voltage = ((data[i * 2] << 8) | data[i * 2 + 1]);
        // The voltage is in uV, Reverse Cell's voltage from descendant to ascendent
        voltage[nbr_of_cells_to_read - 1 - i] = (((float)temp_voltage) * BQ75614_ADC_RESOLUTION) / 1000.0f;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the voltage in Volts of a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param cell The cell number to read the voltage from. It should be between 1 and 16
 * @param voltage The buffer to store the voltage in Volts
 * @return BQ75614_StatusType BQ75614_OK if the voltage was read successfully
 */
BQ75614_StatusType BQ75614_ReadCellVoltage(BQ75614_HandleTypeDef *bq75614, uint8_t cell, float *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // We must first certify that the required cell is in the range of the active cells
    uint8_t nbr_of_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &nbr_of_cells));
    // We must first certify that cell is in available range
    if (cell < 1 || cell > nbr_of_cells)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    CHECK_BQ75614_ERROR(ReadCellsVoltage(bq75614, cell, 1, voltage));

    return BQ75614_OK;
}

/**
 * @brief This function returns the voltage in Volts of all the active cells
 *
 * @param bq75614 The BQ75614 handle
 * @param voltage The buffer to store the voltages in mVolts. The buffer must be at least of size `active_cells`
 * @return BQ75614_StatusType BQ75614_OK if the voltages were read successfully
 */
BQ75614_StatusType BQ75614_ReadCellsVoltage(BQ75614_HandleTypeDef *bq75614, float *voltage)
{
    CHECK_NULL(bq75614, voltage);
    // We must first certify that the required cell is in the range of the active cells
    uint8_t nbr_of_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &nbr_of_cells));
    CHECK_BQ75614_ERROR(ReadCellsVoltage(bq75614, 1, nbr_of_cells, voltage));
    
    return BQ75614_OK;
}

/**
 * @brief This function returns the sum in in Volts of all the active cells voltages
 *
 * @param bq75614 The BQ75614 handle
 * @param voltage The buffer to store the sum of the voltages in Volts
 * @return BQ75614_StatusType BQ75614_OK if the sum of the voltages was read successfully
 *
 * @note The sum of the voltages is the sum of all the active cells voltages, not a real measurement.
 */
BQ75614_StatusType BQ75614_ReadPackVoltage(BQ75614_HandleTypeDef *bq75614, float *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // We must first get the available amount of active cells
    uint8_t nbr_of_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &nbr_of_cells));

    float voltages[nbr_of_cells];
    CHECK_BQ75614_ERROR(BQ75614_ReadCellsVoltage(bq75614, voltages)); // Read all the cells voltages in mV

    *voltage = 0;
    for (int i = 0; i < nbr_of_cells; i++)
    {
        *voltage += voltages[i];
    }
    // Convert the sum in mV to V
    *voltage /= 1000.0f;

    return BQ75614_OK;
}

/**
 * @brief Returns the temperature of the die 1 in °C.
 *
 * @param bq75614 The BQ75614 handle
 * @param temperature The buffer to store the temperature in °C
 * @return BQ75614_StatusType BQ75614_OK if the temperature was read successfully
 */
BQ75614_StatusType BQ75614_GetDie1Temperature(BQ75614_HandleTypeDef *bq75614, float *temperature)
{
    CHECK_NULL(bq75614, temperature);

    // Reads the DIETEMP1_HI/LO registers to get the temperature of the die 1
    // Value is in 2s complement format and 0°C is 0x0000. LSB is 0.025°C
    uint8_t data[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DIETEMP1_HI, data, 2));

    // Convert the data to a 16 bits integer
    int16_t temp = (data[0] << 8) | data[1];
    // Convert the temperature to °C
    *temperature = temp * BQ75614_ADC_DIE_TEMPERATURE_RESOLUTION;

    return BQ75614_OK;
}

/**
 * @brief Returns the temperature of the die 2 in °C.
 *
 * @param bq75614 The BQ75614 handle
 * @param temperature The buffer to store the temperature in °C
 * @return BQ75614_StatusType BQ75614_OK if the temperature was read successfully
 */
BQ75614_StatusType BQ75614_GetDie2Temperature(BQ75614_HandleTypeDef *bq75614, float *temperature)
{
    CHECK_NULL(bq75614, temperature);

    // Reads the DIETEMP2_HI/LO registers to get the temperature of the die 2
    // Value is in 2s complement format and 0°C is 0x0000. LSB is 0.025°C
    uint8_t data[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DIETEMP2_HI, data, 2));

    // Convert the data to a 16 bits integer
    int16_t temp = (data[0] << 8) | data[1];
    // Convert the temperature to °C
    *temperature = temp * BQ75614_ADC_DIE_TEMPERATURE_RESOLUTION;

    return BQ75614_OK;
}

/**
 * @brief This function enables both MAIN ADC and CS ADC to measure the current.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the current sense was enabled successfully
 */
BQ75614_StatusType BQ75614_EnableCurrentSense(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We set the decimation ratio of the CS ADC to 0.768ms
    CHECK_BQ75614_ERROR(BQ75614_SetDecimationRatio(bq75614, BQ75614_DECIMATIONRATIO_0_768MS));
    // we configure the main adc in continuous mode
    CHECK_BQ75614_ERROR(BQ75614_ConfigMainADC(bq75614, BQ75614_MAIN_ADC_MODE_CONTINUOUS_RUN, 0));

    return BQ75614_OK;
}

/**
 * @brief Return the current sense from the Main ADC in mA.
 * This function is used to verify if the `BQ75614_GetCurrentSense` is not out of the range.
 * Prefer this last function to retrieve a more precision current sense.
 *
 * @param bq75614 The BQ75614 handle
 * @param current The buffer to store the current in mA
 * @return BQ75614_StatusType BQ75614_OK if the current was read successfully
 *
 * @note The current is calculated from the equation I = U/R, where U is the voltage read from the ADC and R is the shunt resistor value.
 * @note In order to this function to work, the user `MUST` implement the `BQ75614_SHUNT_RESISTOR` macro with the value of the shunt in `mOhm`.
 * @note If user doesn't want to use this function, the user still needs to set the `BQ75614_SHUNT_RESISTOR` macro to 0.
 */
BQ75614_StatusType BQ75614_GetMainADCCurrentSense(BQ75614_HandleTypeDef *bq75614, float *current)
{
    CHECK_NULL(bq75614, current);

    // get value from MAIN_CURRENT_HI/LO registers in 2's complement format. LSB is 30.52uV
    uint8_t data[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_MAIN_CURRENT_HI, data, 2));

    // Convert the data to a 16 bits integer
    int16_t temp = (data[0] << 8) | data[1];
    // Convert the value to mV
    float mV = (temp * BQ75614_ADC_MAIN_CURRENT_RESOLUTION) * 1000.0;
    // Calculate the current from the equation I = U/R
    // BQ75614_SHUNT_RESISTOR is in mOhm and MUST be implemented by the user
    *current = mV / BQ75614_SHUNT_RESISTOR;

    return BQ75614_OK;
}

/**
 * @brief Returns the current sense from the Current Sense ADC in mA.
 * The current sense is calculated from the voltage read from the CS ADC and the shunt resistor value.
 * If value is out of range, the function will return the current sense from the Main ADC and the `BQ75614_ERROR_CS_CURRENT_OUT_OF_RANGE` error.
 *
 * @param bq75614 The BQ75614 handle
 * @param current The buffer to store the current in mA
 * @return BQ75614_StatusType BQ75614_OK if the current was read successfully.
 * `BQ75614_ERROR_CS_CURRENT_OUT_OF_RANGE` if the current is out of the range.
 *
 * @note The current is calculated from the equation I = U/R, where U is the voltage read from the ADC and R is the shunt resistor value.
 * @note In order to this function to work, the user `MUST` implement the `BQ75614_SHUNT_RESISTOR_USER` macro with the value of the shunt in `mOhm` in the file `bq75614_defines.h`.
 * @note If user doesn't want to use this function, the user still needs to set the `BQ75614_SHUNT_RESISTOR_USER` macro to `0`.
 * @note The current is checked if it is out of the range. If it is, the function will return the current from the Main ADC.
 * @note Indeed, CS ADC can only detect voltage from -125mV to 125mV. A comparison to MAIN ADC is done to check if the current is in the range.
 */
BQ75614_StatusType BQ75614_GetCurrentSense(BQ75614_HandleTypeDef *bq75614, float *current)
{
    CHECK_NULL(bq75614, current);

    // Reads value from CURRENT_HI/MID/LO registers in 2's complement format. LSB is 14.9 nV
    uint8_t data[3] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CURRENT_HI, data, 3));
    // Convert the data to a 24 bits integer
    int32_t temp = (data[0] << 16) | (data[1] << 8) | data[2];
    temp = (temp << 8) >> 8; // Sign extend the 24 bits integer
    // Convert the value to mV
    float mV = (temp * BQ75614_ADC_CS_CURRENT_RESOLUTION) * 1000.0;
    // Calculate the current from the equation I = U/R
    // BQ75614_SHUNT_RESISTOR_USER is in mOhm and MUST be implemented by the user
    *current = mV / BQ75614_SHUNT_RESISTOR;

    // Check if the current is out of the range
    // Retrieve the current from the main ADC
    float main_current;
    CHECK_BQ75614_ERROR(BQ75614_GetMainADCCurrentSense(bq75614, &main_current));

    // Compute main ADC current error range
    // U = RI, I = U/R, I = BQ75614_ADC_MAIN_CURRENT_RESOLUTION / BQ75614_SHUNT_RESISTOR
    float error_range = BQ75614_ADC_MAIN_CURRENT_RESOLUTION / BQ75614_SHUNT_RESISTOR;

    // If current from CS ADC is smaller or greater than MAIN ADC +/- error_range, return main current with error
    if (*current < (main_current - error_range) || *current > (main_current + error_range))
    {
        *current = main_current;
        return BQ75614_ERROR_CS_CURRENT_OUT_OF_RANGE;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets the decimation ratio of CS ADC.
 * The decimation ratio is used to reduce the noise of the CS ADC with the SINC3/DR filter.
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The decimation ratio to set. IT should be of type `BQ75614_DecimationRatioType`
 * @return BQ75614_StatusType BQ75614_OK if the decimation ratio was set successfully
 */
BQ75614_StatusType BQ75614_SetDecimationRatio(BQ75614_HandleTypeDef *bq75614, BQ75614_DecimationRatioType ratio)
{
    CHECK_NULL(bq75614);

    // Test range of ratio
    if (ratio < BQ75614_DECIMATIONRATIO_0_768MS || ratio > BQ75614_DECIMATIONRATIO_12_288MS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Set the bits ADC_CTRL1[CS_DR]
    adc_ctrl1 &= ~(0x3 << ADC_CTRL1_CS_DR_POS);
    adc_ctrl1 |= (ratio << ADC_CTRL1_CS_DR_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that value was correctly written
    BQ75614_DecimationRatioType ratio_read;
    CHECK_BQ75614_ERROR(BQ75614_GetDecimationRatio(bq75614, &ratio_read));
    if (ratio_read != ratio)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the decimation ratio of CS ADC.
 * The decimation ratio is used to reduce the noise of the CS ADC with the SINC3/DR filter.
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The buffer to store the decimation ratio. It should be of type `BQ75614_DecimationRatioType`
 * @return BQ75614_StatusType BQ75614_OK if the decimation ratio was read successfully
 */
BQ75614_StatusType BQ75614_GetDecimationRatio(BQ75614_HandleTypeDef *bq75614, BQ75614_DecimationRatioType *ratio)
{
    CHECK_NULL(bq75614, ratio);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    *ratio = (adc_ctrl1 >> ADC_CTRL1_CS_DR_POS) & 0x3;

    return BQ75614_OK;
}

/**
 * @brief This function configures and starts the Main ADC to measure the following parameters:
 * - Die temperature 1
 * - TSREF -> Reference voltage for thermistor
 * - VC1 - VC16 -> Cell voltages
 * - Main current sense through SRP-SRN shunt resistor
 * - Multiplexed GPIOs
 *
 * @param bq75614 The BQ75614 handle
 * @param mode The mode to set. Of type `BQ75614_MainADCModeType`
 * @param delay The delay to wait before starting the Main ADC. between 0 and 200 us in steps of 5 us
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC was configured successfully
 *
 * @note Active cells must be configured before calling this function
 */
BQ75614_StatusType BQ75614_ConfigMainADC(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType mode, uint8_t delay)
{
    CHECK_NULL(bq75614);
    // Verify that mode is in the right range
    if (mode > BQ75614_MAIN_ADC_MODE_CONTINUOUS_RUN)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Set the Main ADC mode
    CHECK_BQ75614_ERROR(BQ75614_SetMainADCMode(bq75614, mode));
    // Set the delay before starting the Main ADC
    CHECK_BQ75614_ERROR(BQ75614_SetMainADCDelay(bq75614, delay));
    // Enable the Main ADC
    CHECK_BQ75614_ERROR(BQ75614_EnableMainADC(bq75614));

    return BQ75614_OK;
}

/**
 * @brief This function enables the Main ADC to measure following parameters:
 * - Die temperature 1
 * - TSREF -> Reference voltage for thermistor
 * - VC1 - VC16 -> Cell voltages
 * - Main current sense through SRP-SRN shunt resistor
 * - Multiplexed GPIOs
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC was enabled successfully
 *
 * @note Config parameters should be previously done
 */
BQ75614_StatusType BQ75614_EnableMainADC(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Set the bit ADC_CTRL1[CS_MAIN_GO]
    adc_ctrl1 |= (1 << ADC_CTRL1_CS_MAIN_GO_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that MAIN ADC is running
    uint8_t enabled;
    CHECK_BQ75614_ERROR(BQ75614_IsMainADCEnabled(bq75614, &enabled));
    if (!enabled)
    {
        return BQ75614_ERROR_MAIN_ADC_NOT_RUNNING;
    }

    // Wait 1 ms to ensure that at least one measurement is done (we could probably wait less, but let's be safe)
    BQ75614_Delay(1);

    return BQ75614_OK;
}

/**
 * @brief This function disables the Main ADC
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC was disabled successfully
 */
BQ75614_StatusType BQ75614_DisableMainADC(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Clear the bit ADC_CTRL1[CS_MAIN_GO]
    adc_ctrl1 &= ~(1 << ADC_CTRL1_CS_MAIN_GO_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that MAIN ADC is not running
    uint8_t enabled;
    CHECK_BQ75614_ERROR(BQ75614_IsMainADCEnabled(bq75614, &enabled));
    if (enabled)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of the Main ADC
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the state of the Main ADC. 1 if enabled, 0 otherwise
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC state was read successfully
 */
BQ75614_StatusType BQ75614_IsMainADCEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    uint8_t dev_stat_read;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_STAT, &dev_stat_read, 1));

    *enabled = (dev_stat_read >> DEV_STAT_MAIN_RUN_POS) & 0x1;

    return BQ75614_OK;
}

/**
 * @brief Set the Main ADC mode. Values can be either `Stop`, `Run for 8 round robin cycles` or `Continuous mode`.
 * The BQ75614 must be enabled again to take value into account.
 *
 * @param bq75614 The BQ75614 handle
 * @param mode The mode to set. Of type `BQ75614_MainADCModeType`
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC mode was set successfully
 *
 * @note Run for 8 round robin cycles will run all multiplexed values 8 times to go around the 8 GPIOs.
 * @note See `Figure 8-3 Main ADC Round Robin Measurements` in the datasheet
 */
BQ75614_StatusType BQ75614_SetMainADCMode(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType mode)
{
    CHECK_NULL(bq75614);

    // We verify that mode is in the right range
    if (mode > BQ75614_MAIN_ADC_MODE_CONTINUOUS_RUN)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Set the bits ADC_CTRL1[CS_MAIN_MODE]
    adc_ctrl1 &= ~(0x3 << ADC_CTRL1_CS_MAIN_MODE_POS);
    adc_ctrl1 |= (mode << ADC_CTRL1_CS_MAIN_MODE_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that mode was correctly set
    BQ75614_MainADCModeType mode_read;
    CHECK_BQ75614_ERROR(BQ75614_GetMainADCMode(bq75614, &mode_read));
    if (mode_read != mode)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the Main ADC mode. Values can be either `Stop`, `Run for 8 round robin cycles` or `Continuous mode`.
 *
 * @param bq75614 The BQ75614 handle
 * @param mode The buffer to store the mode. Of type `BQ75614_MainADCModeType`
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC mode was read successfully
 */
BQ75614_StatusType BQ75614_GetMainADCMode(BQ75614_HandleTypeDef *bq75614, BQ75614_MainADCModeType *mode)
{
    CHECK_NULL(bq75614, mode);
    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    *mode = (adc_ctrl1 >> ADC_CTRL1_CS_MAIN_MODE_POS) & 0x3;

    return BQ75614_OK;
}

/**
 * @brief Set the Main ADC delay. Values can be between 0 and 200 us in 5 us step.
 *
 * @param bq75614 The BQ75614 handle
 * @param delay The delay to set
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC delay was set successfully
 */
BQ75614_StatusType BQ75614_SetMainADCDelay(BQ75614_HandleTypeDef *bq75614, uint8_t delay)
{
    CHECK_NULL(bq75614);
    // we first check if the delay is in the range
    if (delay > 200 || delay % 5 != 0)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We then convert this delay to the register value
    uint8_t delay_reg = delay / 5;

    // Read the register ADC_CONF2
    uint8_t adc_conf2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF2, &adc_conf2, 1));

    // Set the bits ADC_CONF2[ADC_DLY]
    adc_conf2 &= ~(0x3F << ADC_CONF2_ADC_DLY_POS);
    adc_conf2 |= (delay_reg << ADC_CONF2_ADC_DLY_POS);

    // Write the register ADC_CONF2
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CONF2, &adc_conf2, 1));

    // Check that delay was correctly set
    uint8_t delay_read;
    CHECK_BQ75614_ERROR(BQ75614_GetMainADCDelay(bq75614, &delay_read));
    if (delay_read != delay)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the Main ADC delay. Values can be between 0 and 200 us in 5 us step.
 *
 * @param bq75614 The BQ75614 handle
 * @param delay The buffer to store the delay
 * @return BQ75614_StatusType BQ75614_OK if the Main ADC delay was read successfully
 */
BQ75614_StatusType BQ75614_GetMainADCDelay(BQ75614_HandleTypeDef *bq75614, uint8_t *delay)
{
    CHECK_NULL(bq75614, delay);

    // Read the register ADC_CONF2
    uint8_t adc_conf2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF2, &adc_conf2, 1));

    *delay = ((adc_conf2 >> ADC_CONF2_ADC_DLY_POS) & 0x3F) * 5;
    // We set undefined values to 0us
    if (*delay > 200)
    {
        *delay = 0;
    }

    return BQ75614_OK;
}

/**
 * @brief This function enables the LowPassFilter for the Current Sense through MAIN ADC using the Shunt Resistor.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was enabled successfully
 *
 * @note The Low pass filter is only available for the Main ADC current sense, not the CS ADC
 */
BQ75614_StatusType BQ75614_EnableSRLowPassFilter(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Set the bit ADC_CTRL1[LPF_SR_EN]
    adc_ctrl1 |= (1 << ADC_CTRL1_LPF_SR_EN_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that low pass filter is enabled
    uint8_t enabled;
    CHECK_BQ75614_ERROR(BQ75614_IsSRLowPassFilterEnabled(bq75614, &enabled));
    if (!enabled)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function disables the LowPassFilter for the Current Sense through MAIN ADC using the Shunt Resistor
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was disabled successfully
 *
 * @note The Low pass filter is only available for the Main ADC current sense, not the CS ADC
 */
BQ75614_StatusType BQ75614_DisableSRLowPassFilter(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Clear the bit ADC_CTRL1[LPF_SR_EN]
    adc_ctrl1 &= ~(1 << ADC_CTRL1_LPF_SR_EN_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that low pass filter is disabled
    uint8_t enabled;
    CHECK_BQ75614_ERROR(BQ75614_IsSRLowPassFilterEnabled(bq75614, &enabled));
    if (enabled)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of the LowPassFilter for the Current Sense through MAIN ADC using the Shunt Resistor
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the state of the LowPassFilter. 1 if enabled, 0 otherwise
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter state was read successfully
 *
 * @note The Low pass filter is only available for the Main ADC current sense, not the CS ADC
 */
BQ75614_StatusType BQ75614_IsSRLowPassFilterEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    *enabled = (adc_ctrl1 >> ADC_CTRL1_LPF_SR_EN_POS) & 0x1;

    return BQ75614_OK;
}

/**
 * @brief This function sets the LowPassFilter cutoff frequency for the Current Sense through MAIN ADC using the Shunt Resistor
 *
 * @param bq75614 The BQ75614 handle
 * @param filter The filter to set. Of type `BQ75614_LPFType`
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was set successfully
 *
 * @note The Low pass filter is only available for the Main ADC current sense, not the CS ADC
 */
BQ75614_StatusType BQ75614_SetSRLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType filter)
{
    CHECK_NULL(bq75614);

    // We first check if the filter is in the right range
    if (filter > BQ75614_LPF_600HZ)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Set LPF value in ADC_CONF1[LPF_SR]
    // Read the register ADC_CONF1
    uint8_t adc_conf1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    // Clear the bits ADC_CONF1[LPF_SR]
    adc_conf1 &= ~(0x7 << ADC_CONF1_LPF_SR_POS);
    // Set the bits ADC_CONF1[LPF_SR]
    adc_conf1 |= (filter << ADC_CONF1_LPF_SR_POS);

    // Write the register ADC_CONF1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    // Check that value was correctly written
    uint8_t adc_conf1_read;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1_read, 1));
    if (((adc_conf1_read >> ADC_CONF1_LPF_SR_POS) & 0x7) != filter)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the cutoff frequency of the LowPassFilter for the Current Sense through MAIN ADC using the Shunt Resistor
 *
 * @param bq75614 The BQ75614 handle
 * @param filter The buffer to store the state of the LowPassFilter. Of type `BQ75614_LPFType`
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter state was read successfully
 *
 * @note The Low pass filter is only available for the Main ADC current sense, not the CS ADC
 */
BQ75614_StatusType BQ75614_GetSRLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType *filter)
{
    CHECK_NULL(bq75614, filter);

    // Read the register ADC_CONF1
    uint8_t adc_conf1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    *filter = (adc_conf1 >> ADC_CONF1_LPF_SR_POS) & 0x7;
    // If cutoff frequency is larger than 600Hz, we set it to 240Hz
    if (*filter > BQ75614_LPF_600HZ)
    {
        *filter = BQ75614_LPF_240HZ;
    }

    return BQ75614_OK;
}

/**
 * @brief This function enables the LowPassFilter for the Voltage Cell through MAIN ADC
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was enabled successfully
 */
BQ75614_StatusType BQ75614_EnableVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Set the bit ADC_CTRL1[LPF_VCELL_EN]
    adc_ctrl1 |= (1 << ADC_CTRL1_LPF_VCELL_EN_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that value was correctly written
    uint8_t adc_ctrl1_read;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1_read, 1));
    if ((adc_ctrl1_read & (1 << ADC_CTRL1_LPF_VCELL_EN_POS)) == 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function disables the LowPassFilter for the Voltage Cell through MAIN ADC
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was disabled successfully
 */
BQ75614_StatusType BQ75614_DisableVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Clear the bit ADC_CTRL1[LPF_VCELL_EN]
    adc_ctrl1 &= ~(1 << ADC_CTRL1_LPF_VCELL_EN_POS);

    // Write the register ADC_CTRL1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    // Check that value was correctly written
    uint8_t adc_ctrl1_read;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1_read, 1));
    if ((adc_ctrl1_read & (1 << ADC_CTRL1_LPF_VCELL_EN_POS)) != 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of the LowPassFilter for the Voltage Cell through MAIN ADC
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the state of the LowPassFilter. 1 if enabled, 0 otherwise
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter state was read successfully
 */
BQ75614_StatusType BQ75614_IsVCELLLowPassFilterEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the register ADC_CTRL1
    uint8_t adc_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CTRL1, &adc_ctrl1, 1));

    *enabled = (adc_ctrl1 >> ADC_CTRL1_LPF_VCELL_EN_POS) & 0x1;

    return BQ75614_OK;
}

/**
 * @brief This function sets the LowPassFilter cutoff frequency for the Voltage Cell through MAIN ADC
 *
 * @param bq75614 The BQ75614 handle
 * @param filter The filter to set. Of type `BQ75614_LPFType`
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter was set successfully
 */
BQ75614_StatusType BQ75614_SetVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType filter)
{
    CHECK_NULL(bq75614);

    // We first check if the filter is in the right range
    if (filter > BQ75614_LPF_600HZ)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // Set LPF value in ADC_CONF1[LPF_VCELL]
    // Read the register ADC_CONF1
    uint8_t adc_conf1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    // Clear the bits ADC_CONF1[LPF_VCELL]
    adc_conf1 &= ~(0x7 << ADC_CONF1_LPF_VCELL_POS);
    // Set the bits ADC_CONF1[LPF_VCELL]
    adc_conf1 |= (filter << ADC_CONF1_LPF_VCELL_POS);

    // Write the register ADC_CONF1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    // Check that value was correctly written
    uint8_t adc_conf1_read;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1_read, 1));
    if (((adc_conf1_read >> ADC_CONF1_LPF_VCELL_POS) & 0x7) != filter)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the cutoff frequency of the LowPassFilter for the Voltage Cell through MAIN ADC
 *
 * @param bq75614 The BQ75614 handle
 * @param filter The buffer to store the state of the LowPassFilter. Of type `BQ75614_LPFType`
 * @return BQ75614_StatusType BQ75614_OK if the LowPassFilter state was read successfully
 */
BQ75614_StatusType BQ75614_GetVCELLLowPassFilter(BQ75614_HandleTypeDef *bq75614, BQ75614_LPFType *filter)
{
    CHECK_NULL(bq75614, filter);

    // Read the register ADC_CONF1
    uint8_t adc_conf1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ADC_CONF1, &adc_conf1, 1));

    *filter = (adc_conf1 >> ADC_CONF1_LPF_VCELL_POS) & 0x7;
    // If cutoff frequency is larger than 600Hz, we set it to 240Hz
    if (*filter > BQ75614_LPF_600HZ)
    {
        *filter = BQ75614_LPF_240HZ;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of a specific GPIO.
 *
 * @param bq75614 The BQ75614 handle
 * @param gpio The GPIO to get the state from
 * @param state The buffer to store the state of the GPIO
 * @return BQ75614_StatusType BQ75614_OK if the GPIO state was read successfully
 *
 * @note See `BQ75614_GPIOConf` type to know the possible states of a GPIO
 */
BQ75614_StatusType BQ75614_GetGPIOConfig(BQ75614_HandleTypeDef *bq75614, BQ75614_GPIO gpio, BQ75614_GPIOConf *config)
{
    CHECK_NULL(bq75614, config);

    // TODO : Check if gpio is in the right range

    // Be aware that param gpio starts from 0 !!
    // Register GPIO_CONF1[2:0] = GPIO1[2:0] and GPIO_CONF1[5:3] = GPIO2[2:0]
    // Register GPIO_CONF2[2:0] = GPIO3[2:0] and GPIO_CONF2[5:3] = GPIO4[2:0]
    // Register GPIO_CONF3[2:0] = GPIO5[2:0] and GPIO_CONF3[5:3] = GPIO6[2:0]
    // Register GPIO_CONF4[2:0] = GPIO7[2:0] and GPIO_CONF4[5:3] = GPIO8[2:0]

    uint8_t reg = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_GPIO_CONF1 + (gpio / 2), &reg, 1));

    // If GPIO is even, we must get the 3 LSB of the register
    if ((gpio & 1) == 0)
    {
        *config = reg & 0x7;
    }
    else
    {
        *config = (reg >> 3) & 0x7;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets a GPIO config to a specific GPIO
 *
 * @param bq75614 The BQ75614 handle
 * @param gpio The GPIO to set the config to
 * @param config The config to set to the GPIO
 * @return BQ75614_StatusType BQ75614_OK if the GPIO config was set successfully
 *
 * @note Be aware of special config chap 8.3.5 of datasheet
 */
BQ75614_StatusType BQ75614_SetGPIOConfig(BQ75614_HandleTypeDef *bq75614, BQ75614_GPIO gpio, BQ75614_GPIOConf config)
{
    CHECK_NULL(bq75614);

    // We verify that config is in the right range
    if (config > BQ75614_GPIO_ADC_WEAK_PULLDOWN)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // TODO : Check if gpio is in the right range

    // Be aware that param gpio starts from 0 !!
    // Register GPIO_CONF1[2:0] = GPIO1[2:0] and GPIO_CONF1[5:3] = GPIO2[2:0]
    // Register GPIO_CONF2[2:0] = GPIO3[2:0] and GPIO_CONF2[5:3] = GPIO4[2:0]
    // Register GPIO_CONF3[2:0] = GPIO5[2:0] and GPIO_CONF3[5:3] = GPIO6[2:0]
    // Register GPIO_CONF4[2:0] = GPIO7[2:0] and GPIO_CONF4[5:3] = GPIO8[2:0]

    uint8_t reg = 0;
    uint8_t gpio_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_GPIO_CONF1 + (gpio / 2), &reg, 1));

    // If GPIO is even, we must set the 3 LSB of the register
    if ((gpio & 1) == 0)
    {
        gpio_conf = reg & ~(0x7);
        gpio_conf |= config;
    }
    else
    {
        gpio_conf = reg & ~(0x7 << 3);
        gpio_conf |= config << 3;
    }

    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_GPIO_CONF1 + (gpio / 2), &gpio_conf, 1));

    // Check that value was correctly written
    BQ75614_GPIOConf gpio_conf_read;
    CHECK_BQ75614_ERROR(BQ75614_GetGPIOConfig(bq75614, gpio, &gpio_conf_read));
    if (gpio_conf_read != config)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets a specific state `config` to all GPIOs selected in `gpios` parameter.
 * Bits 0 to 7 of `gpios` represent GPIO1 to GPIO8.
 *
 * @param bq75614 The BQ75614 handle
 * @param gpios The GPIOs to set the config to (bitfield)
 * @param config The config to set to the GPIOs
 * @return BQ75614_StatusType BQ75614_OK if the GPIOs config was set successfully
 */
BQ75614_StatusType BQ75614_SetGPIOsConfig(BQ75614_HandleTypeDef *bq75614, uint8_t gpios, BQ75614_GPIOConf config)
{
    CHECK_NULL(bq75614);

    // We verify that config is in the right range
    if (config > BQ75614_GPIO_ADC_WEAK_PULLDOWN)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // TODO : Check if gpio is in the right range

    for (int i = 0; i < 8; i++)
    {
        if (gpios & (1 << i))
        {
            CHECK_BQ75614_ERROR(BQ75614_SetGPIOConfig(bq75614, i, config));
        }
    }
    return BQ75614_OK;
}

/**
 * @brief This function returns the overvoltage threshold of the BQ75614 in mV
 * Possible values are from 2700mV to 3000mV, 3500mV to 3800mV, 4175mV to 4475mV with 25mV steps.
 *
 * @param bq75614 The BQ75614 handle
 * @param voltage The buffer to store the overvoltage threshold in mV
 * @return BQ75614_StatusType BQ75614_OK if the overvoltage threshold was read successfully
 */
BQ75614_StatusType BQ75614_GetOverVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // For Overvoltage
    // All settings are at 25-mV steps.
    // 0x02 to 0x0E: range from 2700 mV to 3000 mV
    // 0x12 to 0x1E: range from 3500 mV to 3800 mV
    // 0x22 to 0x2E: range from 4175 mV to 4475 mV
    // All other settings will default to 2700 mV.

    uint8_t ov_thr = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OV_THRESH, &ov_thr, 1));

    if (ov_thr >= 0x02 && ov_thr <= 0x0E)
    {
        *voltage = (ov_thr - 0x02) * 25 + 2700;
    }
    else if (ov_thr >= 0x12 && ov_thr <= 0x1E)
    {
        *voltage = (ov_thr - 0x12) * 25 + 3500;
    }
    else if (ov_thr >= 0x22 && ov_thr <= 0x2E)
    {
        *voltage = (ov_thr - 0x22) * 25 + 4175;
    }
    else
    {
        *voltage = 2700;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the undervoltage threshold of the BQ75614 in mV
 * Possible values are from 1200mV to 3100mV with 50mV steps.
 *
 * @param bq75614 The BQ75614 handle
 * @param voltage The buffer to store the undervoltage threshold in mV
 * @return BQ75614_StatusType BQ75614_OK if the undervoltage threshold was read successfully
 */
BQ75614_StatusType BQ75614_GetUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // For Undervoltage
    // All settings are at 50-mV steps.
    // 0x00 to 0x26: range from 1200 mV to 3100 mV
    // All other settings will default to 3100 mV.

    uint8_t uv_thr = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_UV_THRESH, &uv_thr, 1));

    if (uv_thr <= 0x26)
    {
        *voltage = uv_thr * 50 + 1200;
    }
    else
    {
        *voltage = 3100;
    }

    return BQ75614_OK;
}

/**
 * @brief Sets undervoltage protection threshold
 *
 * @note After a call to this function, the OVUV protection is not restarted. Call BQ75614_StartOVUV to restart the OVUV protection and apply new value.
 *
 * @param bq75614 The BQ75614 handle
 * @param under_voltage possible values are from 1200mV to 3100mV with 50mV steps. All other values will set to 3100mV
 * @return BQ75614_StatusType BQ75614_OK if the UV threshold was set successfully.
 * BQ75614_ERROR_BAD_PARAMETER if the voltage is not in the right range
 */
BQ75614_StatusType BQ75614_SetUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t voltage)
{
    CHECK_NULL(bq75614);

    if (voltage < 1200 || voltage > 3100)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    // For Undervoltage
    // All settings are at 50-mV steps.
    // 0x00 to 0x26: range from 1200 mV to 3100 mV
    // All other settings will default to 3100 mV.

    // Compute UV_THR[5:0] from the under_voltage parameter
    uint8_t uv_thr = (voltage - 1200) / 50;

    // Write the OVUV register
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_UV_THRESH, &uv_thr, 1));

    // Check that value was correctly written
    uint32_t uv_thr_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetUnderVoltage(bq75614, &uv_thr_read));
    if (uv_thr_read != voltage)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Sets overvoltage protection threshold
 *
 * @note After a call to this function, the OVUV protection is not restarted. Call BQ75614_StartOVUV to restart the OVUV protection and apply new value.
 *
 * @param bq75614 The BQ75614 handle
 * @param over_voltage possible values are from 2700mV to 3000mV, 3500mV to 3800mV, 4175mV to 4475mV with 25mV steps. All other values will set 2700mV
 * @return BQ75614_StatusType BQ75614_OK if the OV threshold was set successfully
 */
BQ75614_StatusType BQ75614_SetOverVoltage(BQ75614_HandleTypeDef *bq75614, uint32_t voltage)
{
    // For Overvoltage
    // All settings are at 25-mV steps.
    // 0x02 to 0x0E: range from 2700 mV to 3000 mV
    // 0x12 to 0x1E: range from 3500 mV to 3800 mV
    // 0x22 to 0x2E: range from 4175 mV to 4475 mV
    // All other settings will default to 2700 mV.

    CHECK_NULL(bq75614);

    // Compute OV_THR[5:0] from the over_voltage parameter
    uint8_t ov_thr = 0;
    if (voltage >= 2700 && voltage <= 3000)
    {
        ov_thr = (voltage - 2700) / 25 + 0x02;
    }
    else if (voltage >= 3500 && voltage <= 3800)
    {
        ov_thr = (voltage - 3500) / 25 + 0x12;
    }
    else if (voltage >= 4175 && voltage <= 4475)
    {
        ov_thr = (voltage - 4175) / 25 + 0x22;
    }
    else
    {
        ov_thr = 0x02;
    }

    // Write the OVUV register
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OV_THRESH, &ov_thr, 1));

    // Check that value was correctly written
    uint32_t ov_thr_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetOverVoltage(bq75614, &ov_thr_read));
    // Comparison is only valid if the voltage was in the right range
    if (ov_thr != 0x02 && ov_thr_read != voltage)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns a 16 bits value whom each bit represent if the undervoltage protection is applied to the corresponding cell
 *
 * @param bq75614 The BQ75614 handle
 * @param lock A buffer of which 16 bits representing is each cell has the undervoltage protector disabled or not
 * @return BQ75614_StatusType BQ75614_OK if the disabled undervoltage register was read successfully
 *
 * @note 0 = Undervoltage protection is enabled
 * @note 1 = Undervoltage protection is disabled
 * @note Bit 0 represents cell 1, bit 1 represents cell 2, and so on
 */
BQ75614_StatusType BQ75614_GetDisabledUnderVoltage(BQ75614_HandleTypeDef *bq75614, uint16_t *uv_disabled)
{
    CHECK_NULL(bq75614, uv_disabled);

    uint8_t uv_disabled_to_write[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_UV_DISABLE1, uv_disabled_to_write, 2));

    *uv_disabled = (uv_disabled_to_write[0] << 8) | uv_disabled_to_write[1];

    return BQ75614_OK;
}

/**
 * @brief This function writes the desired value to undervoltage protections cells.
 *
 * @param bq75614 The BQ75614 handle
 * @param uv_disabled A 16 bits value whom each bit represent if the undervoltage protection is applied to the corresponding cell
 * @return BQ75614_StatusType BQ75614_OK if the disabled undervoltage register was written successfully
 *
 * @note 0 = Undervoltage protection is enabled
 * @note 1 = Undervoltage protection is disabled
 * @note Bit 0 represents cell 1, bit 1 represents cell 2, and so on
 */
static BQ75614_StatusType BQ75614_SetUVDisabled(BQ75614_HandleTypeDef *bq75614, BQ75614_Cell uv_cells)
{
    CHECK_NULL(bq75614);

    uint8_t uv_disabled_to_write[2] = {0};
    uv_disabled_to_write[0] = (uv_cells >> 8) & 0xFF;
    uv_disabled_to_write[1] = uv_cells & 0xFF;

    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_UV_DISABLE1, uv_disabled_to_write, 2));

    // Check that value was correctly written
    uint16_t uv_disabled_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetDisabledUnderVoltage(bq75614, &uv_disabled_read));
    if (uv_disabled_read != uv_cells)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function disables the undervoltage protection for the desired cells.
 *
 * @param bq75614 The BQ75614 handle
 * @param uv_disabled A list of type `BQ75614_Cell` representing cells whose undervoltage protection must me disabled.
 * Example : `BQ75614_CELL2 | BQ75614_CELL5` will disable the undervoltage protection for cells 2 and 5
 * @return BQ75614_StatusType BQ75614_OK if the cells protectors were correctly disabled
 *
 * @note Bit 0 represents cell 1, bit 1 represents cell 2, and so on
 */
BQ75614_StatusType BQ75614_DisableUnderVoltage(BQ75614_HandleTypeDef *bq75614, BQ75614_Cell uv_cells_to_disable)
{
    CHECK_NULL(bq75614);

    uint16_t uv_disabled = 0;

    CHECK_BQ75614_ERROR(BQ75614_GetDisabledUnderVoltage(bq75614, &uv_disabled));

    uv_disabled |= (uv_cells_to_disable & 0xFFFF);

    CHECK_BQ75614_ERROR(BQ75614_SetUVDisabled(bq75614, (BQ75614_Cell)uv_disabled));

    // Check that value was correctly written
    uint16_t uv_disabled_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetDisabledUnderVoltage(bq75614, &uv_disabled_read));
    // We check that the cells we wanted to disable are disabled
    if ((uv_disabled_read & uv_cells_to_disable) != uv_cells_to_disable)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function enables the undervoltage protection for the desired cells.
 *
 * @param bq75614 The BQ75614 handle
 * @param uv_cells_to_enable A list of type `BQ75614_Cell` representing cells whose undervoltage protection must me enabled.
 * Example : `BQ75614_CELL2 | BQ75614_CELL5` will enable the undervoltage protection for cells 2 and 5
 * @return BQ75614_StatusType BQ75614_OK if the cells protectors were correctly enabled
 *
 * @note Bit 0 represents cell 1, bit 1 represents cell 2, and so on
 * @note At power up, all undervoltage protections are enabled
 */
BQ75614_StatusType BQ75614_EnableUnderVoltage(BQ75614_HandleTypeDef *bq75614, BQ75614_Cell uv_cells_to_enable)
{
    CHECK_NULL(bq75614);

    uint16_t uv_disabled = 0;

    CHECK_BQ75614_ERROR(BQ75614_GetDisabledUnderVoltage(bq75614, &uv_disabled));

    uv_disabled &= ~(uv_disabled & 0xFFFF);

    CHECK_BQ75614_ERROR(BQ75614_SetUVDisabled(bq75614, (BQ75614_Cell)uv_disabled));

    // Check that value was correctly written
    uint16_t uv_disabled_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetDisabledUnderVoltage(bq75614, &uv_disabled_read));
    // We check that the cells we wanted to enable are enabled
    if ((uv_disabled_read & uv_cells_to_enable) != 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of the OVUV protection
 *
 * @param bq75614 The BQ75614 handle
 * @param state The buffer to store the state of the OVUV protection
 * @return BQ75614_StatusType BQ75614_OK if the OVUV state was read successfully
 *
 * @note 0 = OVUV protection is not running
 * @note 1 = OVUV protection is running
 */
BQ75614_StatusType BQ75614_IsOVUVEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *state)
{
    CHECK_NULL(bq75614, state);

    uint8_t dev_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_STAT, &dev_stat, 1));

    *state = ((dev_stat >> DEV_STAT_OVUV_RUN_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief Starts the OVUV protection
 *
 * @note This function will start the OVUV protection in Round Robin Mode. If the OVUV protection is already running, it will restart it.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the OVUV protection was started successfully
 */
BQ75614_StatusType BQ75614_StartOVUV(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    
    uint8_t ovuv_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OVUV_CTRL, &ovuv_ctrl, 1));
    // Sets GO bit to 1 and MODE to 01 as round robin
    ovuv_ctrl |= (1 << OVUV_CTRL_GO_POS) | (0b01 << OVUV_CTRL_MODE_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OVUV_CTRL, &ovuv_ctrl, 1));

    // Read DEV_STAT[OVUV_RUN] to check if the OVUV protection is running
    uint8_t dev_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsOVUVEnabled(bq75614, &dev_stat));
    if (dev_stat != 1)
    {
        return BQ75614_ERROR_OVUV_NOT_RUNNING;
    }
    return BQ75614_OK;
}

/**
 * @brief Sets overvoltage and undervoltage protection thresholds and start the OVUV protection
 *
 * @note Call to this function will start or restart the OVUV protection.
 * @note It starts the OVUV protection in Round Robin Mode and checks that it started correctly.
 *
 * @param bq75614 The BQ75614 handle
 * @param over_voltage possible values are from 2700mV to 3000mV, 3500mV to 3800mV, 4175mV to 4475mV with 25mV steps. All other values will set 2700mV
 * @param under_voltage possible values are from 1200mV to 3100mV with 50mV steps. All other values will set to 3100mV
 * @return BQ75614_StatusType BQ75614_OK if the OVUV protection was set successfully
 */
BQ75614_StatusType BQ75614_ConfigOVUV(BQ75614_HandleTypeDef *bq75614, uint32_t over_voltage, uint32_t under_voltage)
{
    CHECK_NULL(bq75614);
    
    // Set the OV and UV thresholds
    CHECK_BQ75614_ERROR(BQ75614_SetOverVoltage(bq75614, over_voltage));

    CHECK_BQ75614_ERROR(BQ75614_SetUnderVoltage(bq75614, under_voltage));

    // Start the OVUV protection
    CHECK_BQ75614_ERROR(BQ75614_StartOVUV(bq75614));

    return BQ75614_OK;
}

/**
 * @brief This function enable the TSREF for the BQ75614. It will supply a stable 5V to external NTCs.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the TSREF was enabled successfully
 *
 * @note The TSREF is mandatory for the OTUT protection to work
 */
BQ75614_StatusType BQ75614_EnableTSREF(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Enable the CONTROL2[TSREF_EN] bit
    uint8_t control2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CONTROL2, &control2, 1));
    control2 |= (1 << CONTROL2_TSREF_EN_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_CONTROL2, &control2, 1));

    // Wait at least 380 us for the TSREF to stabilize
    BQ75614_Delay(1);

    // Verify that the TSREF is enabled
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CONTROL2, &control2, 1));
    if ((control2 & (1 << CONTROL2_TSREF_EN_POS)) == 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function disable the TSREF for the BQ75614. It will stop supplying a stable 5V to external NTCs.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the TSREF was disabled successfully
 *
 * @note The TSREF is mandatory for the OTUT protection to work
 */
BQ75614_StatusType BQ75614_DisableTSREF(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Disable the CONTROL2[TSREF_EN] bit
    uint8_t control2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CONTROL2, &control2, 1));
    control2 &= ~(1 << CONTROL2_TSREF_EN_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_CONTROL2, &control2, 1));

    // Verify that the TSREF is disabled
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CONTROL2, &control2, 1));

    if ((control2 & (1 << CONTROL2_TSREF_EN_POS)) != 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the status of the TSREF for the BQ75614.
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the status of the TSREF. 0 = TSREF is disabled, 1 = TSREF is enabled
 * @return BQ75614_StatusType BQ75614_OK if the TSREF status was read successfully
 */
BQ75614_StatusType BQ75614_IsTSREFEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the CONTROL2[TSREF_EN] bit
    uint8_t control2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_CONTROL2, &control2, 1));

    *enabled = ((control2 >> CONTROL2_TSREF_EN_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief This function returns the voltage of the TSREF for the BQ75614.
 *
 * @param bq75614 The BQ75614 handle
 * @param voltage The buffer to store the voltage of the TSREF
 * @return BQ75614_StatusType BQ75614_OK if the TSREF voltage was read successfully
 *
 * @note The TSREF_EN bit must be set to 1 to get a valid value
 */
BQ75614_StatusType BQ75614_GetTSREF(BQ75614_HandleTypeDef *bq75614, float *voltage)
{
    CHECK_NULL(bq75614, voltage);

    // We first check if the TSREF is enabled
    uint8_t enabled = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsTSREFEnabled(bq75614, &enabled));
    if (enabled == 0)
    {
        return BQ75614_ERROR_TSREF_NOT_ENABLED;
    }

    // TODO check if ADC is enabled, if yes cool, otherwise start a single run

    // Read the MAIN ADC result of the TSREF
    uint8_t tsref[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_TSREF_HI, tsref, 2));

    // Compute the voltage
    int16_t tsref_value = (tsref[0] << 8) | tsref[1];
    *voltage = tsref_value * BQ75614_ADC_TSREF_RESOLUTION;

    return BQ75614_OK;
}

/**
 * @brief This function sets the overtemperature threshold of the BQ75614 in ratiometric for all thermistors connected to active GPIOs.
 * The ratio is to be compared to the measured (GPIO voltage / TSREF)
 * For exemple, if we set the ratio to 10%, if the (GPIO voltage / TSREF) < 0.1, it will trigger the fault
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The overtemperature threshold in ratiometric. The value must be between 10% to 39% in steps of 1%
 * @return BQ75614_StatusType BQ75614_OK if the OT threshold was set successfully
 *
 * @note When this function is called. A call to `BQ75614_StartOTUT` is needed to restart the OTUT protection
 */
BQ75614_StatusType BQ75614_SetOverTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t ratio)
{
    CHECK_NULL(bq75614);

    // Test that ratio is in the correct range
    if (ratio < BQ75614_MIN_OT_RATIO || ratio > BQ75614_MAX_OT_RATIO)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // Read actual OTUT_THRESH register
    uint8_t otut_thresh = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    // We clear old OTUT_THRESH[OT_THR4:0] and set the new ratio
    otut_thresh &= ~(0x1F);
    otut_thresh |= (ratio - BQ75614_MIN_OT_RATIO);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    // Verify that the OTUT_THRESH was correctly set
    uint8_t otut_thresh_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetOverTemperature(bq75614, &otut_thresh_read));
    if (otut_thresh_read != ratio)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the overtemperature threshold of the BQ75614 in ratiometric for all thermistors connected to active GPIOs.
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The buffer to store the overtemperature threshold in ratiometric. The value must be between 10% to 39% in steps of 1%
 * @return BQ75614_StatusType BQ75614_OK if the OT threshold was read successfully
 */
BQ75614_StatusType BQ75614_GetOverTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t *ratio)
{
    CHECK_NULL(bq75614, ratio);

    // Read actual OTUT_THRESH register
    uint8_t otut_thresh = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    *ratio = (otut_thresh & 0x1F) + BQ75614_MIN_OT_RATIO;

    // That's writtent on datasheet, but should never happen
    if (*ratio > BQ75614_MAX_OT_RATIO)
    {
        *ratio = BQ75614_MAX_OT_RATIO;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets the undertemperature threshold of the BQ75614 in ratiometric for all thermistors connected to active GPIOs.
 * The ratio is to be compared to the measured (GPIO voltage / TSREF)
 * For exemple, if we set the ratio to 70%, if the (GPIO voltage / TSREF) > 0.7, it will trigger the fault
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The undertemperature threshold in ratiometric. The value must be between 66% and 80% in steps of 2%
 * @return BQ75614_StatusType BQ75614_OK if the UT threshold was set successfully
 *
 * @note When this function is called. A call to `BQ75614_StartOTUT` is needed to restart the OTUT protection
 */
BQ75614_StatusType BQ75614_SetUnderTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t ratio)
{
    CHECK_NULL(bq75614);

    // Test that ratio is in the correct range
    if (ratio < BQ75614_MIN_UT_RATIO || ratio > BQ75614_MAX_UT_RATIO)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // Read actual OTUT_THRESH register
    uint8_t otut_thresh = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    // We clear old OTUT_THRESH[UT_THR2:0] and set the new ratio
    otut_thresh &= ~(0x07 << 5);
    otut_thresh |= ((ratio - BQ75614_MIN_UT_RATIO) / 2) << 5;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    // Verify that the OTUT_THRESH was correctly set
    uint8_t otut_thresh_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetUnderTemperature(bq75614, &otut_thresh_read));
    if (otut_thresh_read != ratio)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the undertemperature threshold of the BQ75614 in ratiometric for all thermistors connected to active GPIOs.
 *
 * @param bq75614 The BQ75614 handle
 * @param ratio The buffer to store the undertemperature threshold in ratiometric. The value must be between 66% and 80% in steps of 2%
 * @return BQ75614_StatusType BQ75614_OK if the UT threshold was read successfully
 */
BQ75614_StatusType BQ75614_GetUnderTemperature(BQ75614_HandleTypeDef *bq75614, uint8_t *ratio)
{
    CHECK_NULL(bq75614, ratio);

    // Read actual OTUT_THRESH register
    uint8_t otut_thresh = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTUT_THRESH, &otut_thresh, 1));

    // We get the bits from 7 to 5 and multiply by 2 to get the ratio
    *ratio = ((otut_thresh >> 5) & 0x07) * 2 + BQ75614_MIN_UT_RATIO;

    return BQ75614_OK;
}

/**
 * @brief This function returns the status of the OTUT protection of the BQ75614. It reads the DEV_STAT[OTUT_RUN] bit.
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the status of the OTUT protection. 0 = OTUT is disabled, 1 = OTUT is enabled
 * @return BQ75614_StatusType BQ75614_OK if the OTUT status was read successfully
 */
BQ75614_StatusType BQ75614_IsOTUTEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the DEV_STAT[OTUT_RUN] bit
    uint8_t dev_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_STAT, &dev_stat, 1));

    *enabled = ((dev_stat >> DEV_STAT_OTUT_RUN_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief This function starts the OTUT protection of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the OTUT protection was started successfully
 *
 * @note This function must be called after setting the OTUT thresholds first
 * @note This function needs TSREF to be enabled
 */
BQ75614_StatusType BQ75614_StartOTUT(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We test that TSREF is enabled
    uint8_t tsref_enabled = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsTSREFEnabled(bq75614, &tsref_enabled));
    if (!tsref_enabled)
    {
        return BQ75614_ERROR_TSREF_NOT_ENABLED;
    }

    // We set the OTUT mode to round robin and to GO
    uint8_t otut_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTUT_CTRL, &otut_ctrl, 1));
    otut_ctrl |= (PROTECTION_ROUND_ROBIN_RUN_MODE << OTUT_CTRL_MODE_POS) | (1 << OTUT_CTRL_GO_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OTUT_CTRL, &otut_ctrl, 1));

    // Verify that the OTUT protection is running
    uint8_t is_OTUT_enabled = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsOTUTEnabled(bq75614, &is_OTUT_enabled));

    if (!is_OTUT_enabled)
    {
        return BQ75614_ERROR_OTUT_NOT_RUNNING;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets the overtemperautre and undertemperature threshold of the BQ75614 in °C for all thermistors connected to active GPIOs.
 *
 * @param bq75614 The BQ75614 handle
 * @param over_temperature The overtemperature threshold in ratiometric. The value must be between 10% to 39% in steps of 1%
 * @param under_temperature The undertemperature threshold in ratiometric. The value must be between 66% and 80% in steps of 2%
 * @param gpios The GPIOs to set the OTUT protection to (bitfield)
 * @return BQ75614_StatusType BQ75614_OK if the OTUT thresholds were set successfully
 *
 * @note This function will set all necessary GPIOs to ADC and OTUT mode
 * @note It will also enable the TSREF for protector use. Otherwise the use of the protection will trigger all OTUT faults
 * @note The OTUT comparator use TSREF as reference.
 * @note A OT fault is detected if (GPIO voltage / TSREF) < OTUT_THRESH[OT_THR4:0]
 * @note A UT fault is detected if (GPIO voltage / TSREF) > OTUT_THRESH[UT_THR2:0]
 */
BQ75614_StatusType BQ75614_ConfigOTUT(BQ75614_HandleTypeDef *bq75614, uint8_t over_temperature_ratio, uint8_t under_temperature_ratio, uint8_t gpios)
{
    CHECK_NULL(bq75614);

    // If the overtemperature ratio is not in the correct range
    if (over_temperature_ratio < BQ75614_MIN_OT_RATIO || over_temperature_ratio > BQ75614_MAX_OT_RATIO)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // If the undertemperature ratio is not in the correct range
    if (under_temperature_ratio < BQ75614_MIN_UT_RATIO || under_temperature_ratio > BQ75614_MAX_UT_RATIO)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We enable TSREF
    CHECK_BQ75614_ERROR(BQ75614_EnableTSREF(bq75614));

    // We parameterize the GPIOs with the ADC and OTUT Mode
    CHECK_BQ75614_ERROR(BQ75614_SetGPIOsConfig(bq75614, gpios, BQ75614_GPIO_INPUT_ADC_OTUT));

    // We set the overtemperature and undertemperature thresholds
    CHECK_BQ75614_ERROR(BQ75614_SetOverTemperature(bq75614, over_temperature_ratio));

    CHECK_BQ75614_ERROR(BQ75614_SetUnderTemperature(bq75614, under_temperature_ratio));

    // We start the OTUT protection
    CHECK_BQ75614_ERROR(BQ75614_StartOTUT(bq75614));

    return BQ75614_OK;
}

/**
 * @brief This function returns the revision ID of the die used by the BQ75614
 *
 * @note 0x10 = Revision A0
 * @note 0x11 = Revision A1
 * @note 0x20 = Revision B0
 * @note 0x21 = Revision B1
 * @note 0x22 = Revision B2
 *
 * @param bq75614 The BQ75614 handle
 * @param die_id The buffer to store the die ID
 * @return BQ75614_StatusType
 */
BQ75614_StatusType BQ75614_GetDIEID(BQ75614_HandleTypeDef *bq75614, uint8_t *die_id)
{
    CHECK_NULL(bq75614, die_id);

    uint8_t data = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DIE_ID1, &data, 1));
    *die_id = data;
    return BQ75614_OK;
}

/**
 * @brief This function returns the part ID of the BQ75614
 *
 * @note 0x03 = BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param part_id The buffer to store the part ID
 * @return BQ75614_StatusType BQ75614_OK if the part ID was read successfully
 */
BQ75614_StatusType BQ75614_GetPARTID(BQ75614_HandleTypeDef *bq75614, uint8_t *part_id)
{
    CHECK_NULL(bq75614, part_id);

    uint8_t data = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_PARTID, &data, 1));
    *part_id = data;
    return BQ75614_OK;
}

/**
 * @brief This function reads the DEV_REVID register of the BQ75614.
 * Although it's name, this register is supposed to have the value 0x00 if the chip is in normal_mode
 * or another value is the Factory Testmode was activated
 * Refer Safety Manual for details on SM426: Fact Testmode Detection
 *
 * @param bq75614
 * @param dev_revid
 * @return BQ75614_StatusType
 */
BQ75614_StatusType BQ75614_GetDEVREVID(BQ75614_HandleTypeDef *bq75614, uint8_t *dev_revid)
{
    CHECK_NULL(bq75614, dev_revid);

    uint8_t data = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_REVID, &data, 1));
    *dev_revid = data;
    return BQ75614_OK;
}

/**
 * @brief Set active number of cells in series. It goes from 6 to 16
 *
 * @param bq75614 The BQ75614 handle
 * @param nbr_of_cells The number of cells in series to set
 * @return BQ75614_StatusType BQ75614_OK if the number of cells was set successfully
 */
BQ75614_StatusType BQ75614_SetActiveCells(BQ75614_HandleTypeDef *bq75614, uint8_t nbr_of_cells)
{
    CHECK_NULL(bq75614);

    // TODO: this is a OTP register, modify in consequence
    if (nbr_of_cells < BQ75614_MIN_ACTIVE_CELLS || nbr_of_cells > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    uint8_t data = nbr_of_cells - BQ75614_MIN_ACTIVE_CELLS; // 6 -> 0x00, 7 -> 0x01, 8 -> 0x02, ..., 16 -> 0x0A
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_ACTIVE_CELL, &data, 1));
    // We then read the register to verify that the value was correctly set
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &data));
    if (data != nbr_of_cells)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }
    return BQ75614_OK;
}

/**
 * @brief Get the active number of cells in series. It goes from 6 to 16
 *
 * @param bq75614 The BQ75614 handle
 * @param nbr_of_cells Buffer for the number of cells in series
 * @return BQ75614_StatusType BQ75614_OK if the number of cells was read successfully
 */
BQ75614_StatusType BQ75614_GetActiveCells(BQ75614_HandleTypeDef *bq75614, uint8_t *nbr_of_cells)
{
    CHECK_NULL(bq75614, nbr_of_cells);

    uint8_t data = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_ACTIVE_CELL, &data, 1));
    *nbr_of_cells = data + BQ75614_MIN_ACTIVE_CELLS;
    return BQ75614_OK;
}

/**
 * @brief This function starts the cell balancing of the BQ75614 by setting the `BAL_CTRL2[BAL_GO]` bit to 1
 *
 * It will then verify that the cell balancing is running by reading the `BAL_STAT[CB_RUN]` bit
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType `BQ75614_OK` if the cell balancing was started successfully
 *
 * @note This function must be called after setting configuration of the cell balancing
 * @note This function will not start the cell balancing if the configuration is invalid and will return `BQ75614_ERROR_BALANCING_CONF_INVALID`
 * @note This function will return `BQ75614_ERROR_BALANCING_NOT_RUNNING` if the cell balancing is not running after the call
 */
BQ75614_StatusType BQ75614_StartBalancing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We set the BAL_CTRL2[BAL_GO] bit to 1
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));
    bal_ctrl2 |= (1 << BAL_CTRL2_BAL_GO_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    // Verify that the cell balancing is running
    uint8_t is_balancing_enabled = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsBalancingEnabled(bq75614, &is_balancing_enabled));

    if (!is_balancing_enabled)
    {
        // We check if the configuration is invalid
        uint8_t is_conf_invalid = 0;
        CHECK_BQ75614_ERROR(BQ75614_IsBalancingConfInvalid(bq75614, &is_conf_invalid));
        if (is_conf_invalid)
        {
            return BQ75614_ERROR_BALANCING_CONF_INVALID;
        }
        else
        {
            return BQ75614_ERROR_BALANCING_NOT_RUNNING;
        }
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the state of the cell balancing of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the state of the cell balancing. 0 = cell balancing is not running, 1 = cell balancing is running
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing state was read successfully
 */
BQ75614_StatusType BQ75614_IsBalancingEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the BAL_STAT[CB_RUN] bit
    uint8_t bal_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_STAT, &bal_stat, 1));

    *enabled = ((bal_stat >> BAL_STAT_CB_RUN_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief This function returns if the state of the cell balancing configuration is invalid
 *
 * @param bq75614 The BQ75614 handle
 * @param invalid The buffer to store the state of the cell balancing configuration invalid. 0 = cell balancing configuration is valid, 1 = cell balancing configuration is invalid
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing configuration invalid state was read successfully
 */
BQ75614_StatusType BQ75614_IsBalancingConfInvalid(BQ75614_HandleTypeDef *bq75614, uint8_t *invalid)
{
    CHECK_NULL(bq75614, invalid);

    // Read the BAL_STAT[CB_CONF_INVALID] bit
    uint8_t bal_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_STAT, &bal_stat, 1));

    *invalid = ((bal_stat >> BAL_STAT_INVALID_CBCONF_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief This function disables the cell balancing of the BQ75614 by setting a `VCB_DONE_THRESH`
 * greater than all actual cell voltages and issuing `BAL_CTRL2[BAL_GO]=1`
 * It will then verify that the cell balancing is not running by checking that `BAL_STAT[CB_DONE]` bit is one
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType `BQ75614_OK` if the cell balancing was stopped successfully
 * @note This function is not implemented yet
 */
BQ75614_StatusType BQ75614_DisableBalancing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // TODO
    return BQ75614_ERROR;
}

/**
 * @brief This function returns if cell balancing is paused by reading the `BAL_STAT[CB_INPAUSE]` bit
 *
 * @param bq75614 The BQ75614 handle
 * @param paused The buffer to store the state of the cell balancing. 0 = cell balancing is not paused, 1 = cell balancing is paused
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing paused state was read successfully
 *
 * @note Reasons for the CB to be paused are:
 * @note - MCU set pause to all a diagnostic running
 * @note - The die tempature exceeds `T_CB_TWARN` = 105 °C
 * @note - OTCB (overtemperature cell balancing) is active and thermistor detects a temperature over OTCB_THR
 */
BQ75614_StatusType BQ75614_IsBalancingPaused(BQ75614_HandleTypeDef *bq75614, uint8_t *paused)
{
    CHECK_NULL(bq75614, paused);

    // Read the BAL_STAT[CB_INPAUSE] bit
    uint8_t bal_stat = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_STAT, &bal_stat, 1));

    *paused = ((bal_stat >> BAL_STAT_CB_INPAUSE_POS) & 1);

    return BQ75614_OK;
}

/**
 * @brief This function pauses the cell balancing of the BQ75614 by setting the `BAL_CTRL2[CB_PAUSE]` bit to 1
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType `BQ75614_OK` if the cell balancing was paused successfully
 *
 * @note One of the reasons to pause manually the cell balancing is to allow a diagnostic
 */
BQ75614_StatusType BQ75614_PauseBalancing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We set the BAL_CTRL2[CB_PAUSE] bit to 1
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));
    bal_ctrl2 |= (1 << BAL_CTRL2_CB_PAUSE_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    // Verify that the cell balancing is paused
    uint8_t is_balancing_paused = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsBalancingPaused(bq75614, &is_balancing_paused));

    if (!is_balancing_paused)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function sets the cell balancing duty cycle of the BQ75614. It defines when balancing goes from odd to even cells
 *
 * @param bq75614 The BQ75614 handle
 * @param duty_cycle Interval between odd and even cell balancing.
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing duty cycle was set successfully
 *
 * @note Only eight cells at maximum can be balanced at a time, that's why there is this duty cycle mechanism
 * @note Duty cycle is effective only when the cell balancing is set to auto with `BAL_CTRL2[AUTO_BAL] = 1`
 */
BQ75614_StatusType BQ75614_SetBalancingDutyCycle(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingDutyCycleType duty_cycle)
{
    CHECK_NULL(bq75614);

    uint8_t bal_ctrl1 = ((duty_cycle & 0x07) << BAL_CTRL1_DUTY_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL1, &bal_ctrl1, 1));

    // Verify that the cell balancing duty cycle was correctly set
    BQ75614_CellBalancingDutyCycleType duty_cycle_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetBalancingDutyCycle(bq75614, &duty_cycle_read));
    if (duty_cycle_read != duty_cycle)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief This function returns the cell balancing duty cycle of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param duty_cycle The buffer to store the cell balancing duty cycle
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing duty cycle was read successfully
 */
BQ75614_StatusType BQ75614_GetBalancingDutyCycle(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingDutyCycleType *duty_cycle)
{
    CHECK_NULL(bq75614, duty_cycle);

    uint8_t bal_ctrl1 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL1, &bal_ctrl1, 1));

    *duty_cycle = (bal_ctrl1 >> BAL_CTRL1_DUTY_POS) & 0x07;

    return BQ75614_OK;
}

/**
 * @brief Get the remaining time in seconds of the cell balancing for a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param cell The cell to get the balancing remaining time
 * @param time The buffer to store the remaining time in seconds of the cell balancing.
 * ´0´ means that there is no remaining time for this cell.
 * Values ´635´ and ´38100´ means that there is a problem with the cell balancing configuration
 *
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing remaining time was read successfully
 */
BQ75614_StatusType BQ75614_GetCellBalancingRemainingTime(BQ75614_HandleTypeDef *bq75614, uint8_t cell, uint32_t *time)
{
    CHECK_NULL(bq75614, time);

    // We check that the cell is in the correct range
    if (cell < BQ75614_MIN_ACTIVE_CELLS || cell > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We set the cell we want to check the remaining time by setting the BAL_CTRL3[BAL_TIME_SEL[3:0]]
    uint8_t bal_ctrl3 = ((cell & 0x0F) << BAL_CTRL3_BAL_TIME_SEL_POS);
    // We add the start flag
    bal_ctrl3 |= (1 << BAL_CTRL3_BAL_TIME_GO_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL3, &bal_ctrl3, 1));

    // We can now read the remaining time and the unit in the BAL_TIME register. Bit 7 is the unit and bits 6:0 are the time
    uint8_t bal_time = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_TIME, &bal_time, 1));

    // Mask unit bit and multiply by the resolution
    *time = bal_time & ~(1 << BAL_TIME_TIME_UNIT_POS) * BQ75614_BAL_TIME_RESOLUTIONS_STEP;

    // Convert all in seconds
    if (bal_time & (1 << BAL_TIME_TIME_UNIT_POS)) // time is in minutes
    {
        *time = *time * 60;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the remaining time in seconds of cell balancing for all active cells
 *
 * @param bq75614 The BQ75614 handle
 * @param time The buffer to store the remaining time for all running cells. time is in seconds
 * ´0´ means that there is no remaining time for this cell.
 * Values ´635´ and ´38100´ means that there is a problem with the cell balancing configuration
 *
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing remaining time was read successfully
 */
BQ75614_StatusType BQ75614_GetCellsBalancingRemainingTime(BQ75614_HandleTypeDef *bq75614, uint32_t *time)
{
    CHECK_NULL(bq75614, time);

    // We get active cells
    uint8_t active_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &active_cells));

    // We loop over all active cells to retrieve the remaining time
    for (uint8_t cell = 0; cell <= active_cells; cell++)
    {
        uint32_t cell_time = 0;
        CHECK_BQ75614_ERROR(BQ75614_GetCellBalancingRemainingTime(bq75614, cell, &cell_time));
        // We store the time in the buffer
        time[cell] = cell_time;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the configured cell balancing timer for a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param starting_cell The cell to get the balancing timer, starting from 1 to 16
 * @param nbr_of_cells_to_read The number of cells to read the balancing timer. Must be between 1 and 16
 * @param timer The buffer to store the cell balancing timer. The buffer must be at least of size `nbr_of_cells_to_read`.
 * timer[0] will contain cell's timer 1, and so on.
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was read successfully
 */
static BQ75614_StatusType GetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t starting_cell, uint8_t nbr_of_cells_to_read, uint8_t *timer)
{
    CHECK_NULL(bq75614, timer);

    if (starting_cell + nbr_of_cells_to_read - 1 > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }
    uint16_t reg = REG_CB_CELL1_CTRL - ((starting_cell - 1) + (nbr_of_cells_to_read - 1));
    uint8_t data[BQ75614_MAX_ACTIVE_CELLS] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, reg, data, nbr_of_cells_to_read));

    // invert the array
    for (uint8_t i = 0; i < nbr_of_cells_to_read; i++)
    {
        timer[i] = data[nbr_of_cells_to_read - 1 - i];
    }

    return BQ75614_OK;
}

/**
 * @brief Set the cell balancing timer for a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param starting_cell The cell to set the balancing timer, starting from 1 to 16
 * @param nbr_of_cells_to_set The number of cells to set the balancing timer. Must be between 1 and 16
 * @param timer The cell balancing timer. All cells will have the same timer.
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was set successfully
 */
static BQ75614_StatusType SetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t starting_cell, uint8_t nbr_of_cells_to_set, uint8_t timer)
{
    CHECK_NULL(bq75614);

    if (starting_cell + nbr_of_cells_to_set - 1 > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    uint16_t reg = REG_CB_CELL1_CTRL - ((starting_cell - 1) + (nbr_of_cells_to_set - 1));

    // fill the array
    for (uint8_t i = 0; i < nbr_of_cells_to_set; i++)
    {
        CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, reg + i, &timer, 1));
    }

    // TODO verify that the value was correctly set

    return BQ75614_OK;
}

/**
 * @brief Get the configured cell balancing timer for a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param cell The cell to get the balancing timer
 * @param timer The buffer to store the cell balancing timer
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was read successfully
 *
 * @note To see the remaining time use `BQ75614_GetCellBalancingRemainingTime`
 */
BQ75614_StatusType BQ75614_GetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t cell, BQ75614_CellBalancingTimerType *timer)
{
    CHECK_NULL(bq75614, timer);

    // We check that the cell is in the correct range
    if (cell > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We get the timer for the cell
    uint8_t timer_read = 0;
    CHECK_BQ75614_ERROR(GetCellBalancingTimer(bq75614, cell, 1, &timer_read));

    *timer = timer_read;

    return BQ75614_OK;
}

/**
 * @brief Set the cell balancing timer for a specific cell
 *
 * @param bq75614 The BQ75614 handle
 * @param cell The cell to set the balancing timer
 * @param timer The cell balancing timer
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was set successfully
 */
BQ75614_StatusType BQ75614_SetCellBalancingTimer(BQ75614_HandleTypeDef *bq75614, uint8_t cell, BQ75614_CellBalancingTimerType timer)
{
    CHECK_NULL(bq75614);

    // We check that the cell is in the correct range
    if (cell > BQ75614_MAX_ACTIVE_CELLS)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We set the timer for the cell
    CHECK_BQ75614_ERROR(SetCellBalancingTimer(bq75614, cell, 1, timer));

    // We verify that the timer was correctly set
    BQ75614_CellBalancingTimerType timer_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetCellBalancingTimer(bq75614, cell, &timer_read));
    if (timer_read != timer)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the configured cell balancing timer for all active cells
 *
 * @param bq75614 The BQ75614 handle
 * @param timer The buffer to store the cell balancing timer for all active cells.
 *  Buffer must be an array to contain all cells value. For each value, bit 7 is the unit (0 = min, 1 = sec), bits 6:0 are the timer
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was read successfully
 */
BQ75614_StatusType BQ75614_GetCellsBalancingTimer(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingTimerType *timer)
{
    CHECK_NULL(bq75614, timer);

    // We get active cells
    uint8_t active_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &active_cells));

    CHECK_BQ75614_ERROR(GetCellBalancingTimer(bq75614, 1, active_cells, (uint8_t*)timer));

    return BQ75614_OK;
}

/**
 * @brief Set the cell balancing timer for all active cells
 *
 * @param bq75614 The BQ75614 handle
 * @param timer The cell balancing timer for all active cells.
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing timer was set successfully
 */
BQ75614_StatusType BQ75614_SetCellsBalancingTimer(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingTimerType timer)
{
    CHECK_NULL(bq75614);
    // We get active cells
    uint8_t active_cells = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetActiveCells(bq75614, &active_cells));

    CHECK_BQ75614_ERROR(SetCellBalancingTimer(bq75614, 1, active_cells, timer));

    // We verify that the timer was correctly set
    BQ75614_CellBalancingTimerType timer_read[BQ75614_MAX_ACTIVE_CELLS] = {0};
    CHECK_BQ75614_ERROR(BQ75614_GetCellsBalancingTimer(bq75614, timer_read));
    for (uint8_t i = 0; i < active_cells; i++)
    {
        if (timer_read[i] != timer)
        {
            return BQ75614_ERROR_REGISTER_NOT_SET;
        }
    }

    return BQ75614_OK;
}

/**
 * @brief Get the cell balancing mode of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param mode The buffer to store the cell balancing mode
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing mode was read successfully
 */
BQ75614_StatusType BQ75614_GetBalancingMode(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingModeType *mode)
{
    CHECK_NULL(bq75614, mode);

    // Read the BAL_CTRL2[BAL_MODE] bit
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    *mode = (bal_ctrl2 >> BAL_CTRL2_AUTO_BAL_POS) & 1;

    return BQ75614_OK;
}

/**
 * @brief Set the cell balancing mode of the BQ75614
 *
 * @param bq75614 The BQ75614 handle
 * @param mode The cell balancing mode to set. `BQ75614_BALANCING_MODE_MANUAL` or `BQ75614_BALANCING_MODE_AUTO`
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing mode was set successfully
 */
BQ75614_StatusType BQ75614_SetBalancingMode(BQ75614_HandleTypeDef *bq75614, BQ75614_CellBalancingModeType mode)
{
    CHECK_NULL(bq75614);
    
    // We set the BAL_CTRL2[BAL_MODE] bit to the desired mode
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));
    bal_ctrl2 &= ~(1 << BAL_CTRL2_AUTO_BAL_POS);   // We clear old value
    bal_ctrl2 |= (mode << BAL_CTRL2_AUTO_BAL_POS); // We set the new value
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    // Verify that the cell balancing mode was correctly set
    BQ75614_CellBalancingModeType mode_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetBalancingMode(bq75614, &mode_read));
    if (mode_read != mode)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the cooloff (hysteresis) for the Overtemperature Cell Balancing feature.
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the hysteresis Cell Balancing feature.
 * @return BQ75614_StatusType BQ75614_OK if the OTCB cooloff was read successfully
 */
BQ75614_StatusType BQ75614_GetOTCBCoolOff(BQ75614_HandleTypeDef *bq75614, uint8_t *cooloff)
{
    CHECK_NULL(bq75614, cooloff);

    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));

    *cooloff = (((otcb_ctrl >> OTCB_THRESH_COOLOFF_POS) & 0x07) * 2 + BQ75614_MIN_OTCB_COOLOFF_RATIO);

    return BQ75614_OK;
}

/**
 * @brief Set the cooloff (hysteresis) for the Overtemperature Cell Balancing feature.
 * When the temperature is above the threshold, the cell balancing is stopped and waits until the temperature is below the `threshold - cooloff`
 *
 * @param bq75614 The BQ75614 handle
 * @param cooloff The cooloff value to set. It must be between 4% to 14% in steps of 2%
 * @return BQ75614_StatusType BQ75614_OK if the cooloff was set successfully
 */
BQ75614_StatusType BQ75614_SetOTCBCoolOff(BQ75614_HandleTypeDef *bq75614, uint8_t cooloff)
{
    CHECK_NULL(bq75614);

    // We check that the cooloff is in the correct range
    if (cooloff < BQ75614_MIN_OTCB_COOLOFF_RATIO || cooloff > BQ75614_MAX_OT_RATIO || cooloff % 2 != 0)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We set the cooloff value
    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));
    otcb_ctrl &= ~(0x07 << OTCB_THRESH_COOLOFF_POS);                                            // We clear old value
    otcb_ctrl |= (((cooloff - BQ75614_MIN_OTCB_COOLOFF_RATIO) / 2) << OTCB_THRESH_COOLOFF_POS); // We set the new value, 4 -> 0x00, 6 -> 0x01, 8 -> 0x02, 10 -> 0x03, 12 -> 0x04, 14 -> 0x05
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));

    // Verify that the cooloff was correctly set
    uint8_t cooloff_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetOTCBCoolOff(bq75614, &cooloff_read));
    if (cooloff_read != cooloff)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Set the Overtemperature Cell Balancing threshold. When the temperature is above this threshold, the cell balancing is paused.
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The threshold ratio from 10% to 24% in steps of 2%
 * @return BQ75614_StatusType BQ75614_OK if the OTCB threshold was set successfully
 */
BQ75614_StatusType BQ75614_SetOTCBThreshold(BQ75614_HandleTypeDef *bq75614, uint8_t threshold)
{
    CHECK_NULL(bq75614);
    
    // We check that the threshold is in the correct range
    if (threshold < BQ75614_MIN_OTCB_THRESHOLD || threshold > BQ75614_MAX_OTCB_THRESHOLD || (threshold % 2 != 0))
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We set the threshold value
    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));
    otcb_ctrl &= ~(0x0F);                                      // We clear old value
    otcb_ctrl |= (threshold - BQ75614_MIN_OTCB_THRESHOLD) / 2; // We set the new value (10 -> 0x00, 12 -> 0x01, etc.)
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));

    // Verify that the threshold was correctly set
    uint8_t threshold_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetOTCBThreshold(bq75614, &threshold_read));
    if (threshold_read != threshold)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the Overtemperature Cell Balancing threshold. When the temperature is above this threshold, the cell balancing is paused.
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The buffer to store the OTCB threshold. The threshold ratio is from 10% to 24% in steps of 2%
 * @return BQ75614_StatusType BQ75614_OK if the OTCB threshold was read successfully
 */
BQ75614_StatusType BQ75614_GetOTCBThreshold(BQ75614_HandleTypeDef *bq75614, uint8_t *threshold)
{
    CHECK_NULL(bq75614, threshold);

    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTCB_THRESH, &otcb_ctrl, 1));

    *threshold = (otcb_ctrl & 0x0F) * 2 + BQ75614_MIN_OTCB_THRESHOLD;

    return BQ75614_OK;
}

/**
 * @brief Get the state of the Overtemperature Cell Balancing feature
 *
 * @param bq75614 The BQ75614 handle
 * @param enabled The buffer to store the state of the OTCB feature. 0 = OTCB is not enabled, 1 = OTCB is enabled
 * @return BQ75614_StatusType BQ75614_OK if the OTCB state was read successfully
 */
BQ75614_StatusType BQ75614_IsOTCBEnabled(BQ75614_HandleTypeDef *bq75614, uint8_t *enabled)
{
    CHECK_NULL(bq75614, enabled);

    // Read the BAL_CTRL2[OTCB_EN] bit
    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &otcb_ctrl, 1));

    *enabled = (otcb_ctrl >> BAL_CTRL2_OTCB_EN_POS) & 1;

    return BQ75614_OK;
}

/**
 * @brief Enable the Overtemperature Cell Balancing feature. It allow to use an external thermal sensor to stop the cell balancing if the temperature is too high.
 * This will enable the OTUT feature
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the OTCB feature was enabled successfully
 */
BQ75614_StatusType BQ75614_EnableOTCB(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We set the OTUT bit to 1
    uint8_t otcb_ctrl = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &otcb_ctrl, 1));
    otcb_ctrl |= (1 << BAL_CTRL2_OTCB_EN_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL2, &otcb_ctrl, 1));

    // Verify that the OTCB was correctly enabled
    uint8_t otcb_enabled = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsOTCBEnabled(bq75614, &otcb_enabled));
    if (!otcb_enabled)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

BQ75614_StatusType BQ75614_IsStopBalanceAtFault(BQ75614_HandleTypeDef *bq75614, uint8_t *stop_at_fault)
{
    CHECK_NULL(bq75614, stop_at_fault);

    // Read the BAL_CTRL2[FLTSTOP_EN] bit
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    *stop_at_fault = ((bal_ctrl2 >> BAL_CTRL2_FLTSTOP_EN_POS) & 1);

    return BQ75614_OK;
}

BQ75614_StatusType BQ75614_StopBalanceAtFault(BQ75614_HandleTypeDef *bq75614, uint8_t stop_at_fault)
{
    CHECK_NULL(bq75614);

    // We set the FLTSTOP_EN bit to 1
    uint8_t bal_ctrl2 = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    if (stop_at_fault)
    {
        // we clear the bit
        bal_ctrl2 &= ~(1 << BAL_CTRL2_FLTSTOP_EN_POS);
    }
    else
    {
        // we set the bit
        bal_ctrl2 |= (stop_at_fault << BAL_CTRL2_FLTSTOP_EN_POS);
    }
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_BAL_CTRL2, &bal_ctrl2, 1));

    // Verify that the register is correctly set
    uint8_t stop_at_fault_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_IsStopBalanceAtFault(bq75614, &stop_at_fault_read));
    if (stop_at_fault_read != stop_at_fault)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief Get the Voltage Cell Balancing Done threshold. When the cell voltage is under this threshold, the cell balancing is stopped.
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The buffer to store the threshold voltage to stop the cell balancing. Range from 2450mV to 4000V in steps of 25mV. 0 to disable this feature
 * @return BQ75614_StatusType BQ75614_OK if the VCB Done threshold was read successfully
 *
 * @note The doc says `0x00 = Disables voltage based on CB_DONE comparison`. Don't understand what it means
 */
BQ75614_StatusType BQ75614_GetVCBDoneThreshold(BQ75614_HandleTypeDef *bq75614, uint16_t *threshold)
{
    CHECK_NULL(bq75614, threshold);

    uint8_t vcb_done_thresh = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_VCB_DONE_THRESH, &vcb_done_thresh, 1));

    if (vcb_done_thresh == 0)
    { // feature disabled
        *threshold = 0;
    }
    else
    {
        *threshold = ((vcb_done_thresh - 1) & 0x3f) * 25 + BQ75614_MIN_VCB_DONE_THRESHOLD;
    }

    return BQ75614_OK;
}

/**
 * @brief Sets the Voltage Cell Balancing Done threshold. When the cell voltage is under this threshold, the cell balancing is stopped.
 *
 * @param bq75614 The BQ75614 handle
 * @param threshold The threshold voltage to stop the cell balancing. Range from 2450mV to 4000V in steps of 25mV. 0 to disable this feature (I think, doc is unclear)
 * @return BQ75614_StatusType BQ75614_OK if the VCB Done threshold was set successfully
 *
 * @note The doc says `0x00 = Disables voltage based on CB_DONE comparison`. Don't understand what it means
 */
BQ75614_StatusType BQ75614_SetVCBDoneThreshold(BQ75614_HandleTypeDef *bq75614, uint16_t threshold)
{
    
    CHECK_NULL(bq75614);

    // We check that the threshold is in the correct range. Between range or equal to 0
    if (!((threshold > BQ75614_MIN_VCB_DONE_THRESHOLD && threshold < BQ75614_MAX_VCB_DONE_THRESHOLD) || threshold == 0))
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    // We set the threshold value
    uint8_t vcb_done_thresh = 0;
    if (threshold != 0)
    {
        vcb_done_thresh = (((threshold - BQ75614_MIN_VCB_DONE_THRESHOLD) / 25) & 0x3f) + 1; // We set the new value
    }
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_VCB_DONE_THRESH, &vcb_done_thresh, 1));

    // Verify that the threshold was correctly set
    uint16_t threshold_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetVCBDoneThreshold(bq75614, &threshold_read));
    if ((threshold_read < (threshold - 25)) || (threshold_read > (threshold + 25)))
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }

    return BQ75614_OK;
}

/**
 * @brief
 *
 * @param bq75614 The BQ75614 handle
 * @param timer Time during the cell balancing is active
 * @param auto_mode Manual or Auto mode for cell balancing
 * @param duty_cycle Only used in auto mode. Interval between odd and even cell balancing.
 * @param OTCB_threshold The threshold ratio from 10% to 24% in steps of 2%. When the temperature is above this threshold, the cell balancing is paused.
 * It allows to use an external thermal sensor to stop the cell balancing if the temperature is too high.
 * This will enable the OTUT feature. All thresholds for OTUT `MUST` be set before enabling the OTCB feature
 * Set to 0 to disable this feature
 * @param cooloff Only if `OTCB_threshold` is non zero. Range of temperature to wait before restarting the cell balancing after an OTCB event.
 * From 4% to 14% in steps of 2%.
 * @param vcb_done_threshold The voltage threshold to stop the cell balancing if cell voltage is under this threshold. 0 to disable this feature.
 * You `must` ensure that OVUV is configured and enabled to use this feature. Range from 2450mV to 4000V in steps of 25mV.
 * 0 to disable this feature (I think, doc is unclear)
 * @param stop_at_fault Stop the cell balancing if a fault is detected. 0 to disable this feature, 1 to enable it
 * @return BQ75614_StatusType BQ75614_OK if the cell balancing configuration was set successfully
 *
 * @note All unused arguments can be set to 0. They will not be ignored anyway
 * @note Config of OTUT must be done before enabling the `OTCB feature`
 */
BQ75614_StatusType BQ75614_ConfigBalancing(BQ75614_HandleTypeDef *bq75614,
                                           BQ75614_CellBalancingTimerType timer,
                                           BQ75614_CellBalancingModeType auto_mode,
                                           BQ75614_CellBalancingDutyCycleType duty_cycle,
                                           uint8_t OTCB_threshold,
                                           uint8_t cooloff,
                                           uint16_t vcb_done_threshold,
                                           uint8_t stop_at_fault)
{
    
    CHECK_NULL(bq75614);
    
    // We must first define which channel to enable for cell balancing by setting a timer different from zero
    CHECK_BQ75614_ERROR(BQ75614_SetCellsBalancingTimer(bq75614, timer));

    // We then set the cell balancing mode
    CHECK_BQ75614_ERROR(BQ75614_SetBalancingMode(bq75614, auto_mode));

    // If we are in auto mode, we must set the duty cycle and enable the BAL_CTRL2[AUTO_BAL] bit
    if (auto_mode == BQ75614_BALANCING_MODE_AUTO)
    {
        CHECK_BQ75614_ERROR(BQ75614_SetBalancingDutyCycle(bq75614, duty_cycle));
    }

    // If we want to enable the OTCB feature, we must set the OTUT threshold and the cooloff time
    if (OTCB_threshold != 0)
    {
        CHECK_BQ75614_ERROR(BQ75614_SetOTCBThreshold(bq75614, OTCB_threshold));
        CHECK_BQ75614_ERROR(BQ75614_SetOTCBCoolOff(bq75614, cooloff));

        // We enable the OTCB feature
        CHECK_BQ75614_ERROR(BQ75614_EnableOTCB(bq75614));

        // We start or restart the OTUT feature
        CHECK_BQ75614_ERROR(BQ75614_StartOTUT(bq75614));
    }

    // We set the VCB_DONE threshold
    CHECK_BQ75614_ERROR(BQ75614_SetVCBDoneThreshold(bq75614, vcb_done_threshold));

    if (vcb_done_threshold != 0)
    {
        // We issue a OVUV start to enable the VCB_DONE feature
        CHECK_BQ75614_ERROR(BQ75614_StartOVUV(bq75614));
    }

    CHECK_BQ75614_ERROR(BQ75614_StopBalanceAtFault(bq75614, stop_at_fault));

    // We can now start the cell balancing
    CHECK_BQ75614_ERROR(BQ75614_StartBalancing(bq75614));

    return BQ75614_OK;
}

/**
 * @brief This function will set the bq75614 in sleep mode.
 * Only Cell balancing, OTUT and OVUV will continue to operate. As well as all internal supplies. This function only works if the BQ75614 is in active mode.
 * There is no way to know if the BQ75614 is in sleep mode or not because UART control is down. Only `bq7614->state` will be updated
 * You can wake the BQ75614 with `BQ75614_SleepToActivePing` or `BQ75614_WakeUpPing` (The latter will also reset device)
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if command was sent successfully
 *
 * @note The BQ75614 is put into sleep using the SLEEP bit in the CONTROL1 register
 */
BQ75614_StatusType BQ75614_Sleep(BQ75614_HandleTypeDef *bq75614)
{

    CHECK_NULL(bq75614);

    // test that we are in active mode
    if (bq75614->state != BQ75614_STATE_ACTIVE)
    {
        return BQ75614_ERROR_BAD_STATE;
    }
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_CONTROL1, (uint8_t[]){1 << CONTROL1_SLEEP_POS}, 1));
    bq75614->state = BQ75614_STATE_SLEEP;
    return BQ75614_OK;
}

/**
 * @brief This function will shutdown the BQ75614.
 * All features will be disabled. This function only works if the BQ75614 is in active mode.
 * There is no way to know if the BQ75614 is in sleep mode or not because UART control is down. Only `bq7614->state` will be updated
 * You can wake the BQ75614 with `BQ75614_WakeUpPing`. This will also reset the device.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the command was sent successfully
 *
 * @note The BQ75614 is shutdown using the SHUTDOWN bit in the CONTROL1 register
 */
BQ75614_StatusType BQ75614_Shutdown(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // test that we are in active mode
    if (bq75614->state != BQ75614_STATE_ACTIVE)
    {
        return BQ75614_ERROR_BAD_STATE;
    }
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_CONTROL1, (uint8_t[]){1 << CONTROL1_SHUTDOWN_POS}, 1));
    bq75614->state = BQ75614_STATE_SHUTDOWN;
    return BQ75614_OK;
}

/**
 * @brief This function will proceed to a soft reset of the `BQ75614`
 * It will resets all registers to their OTP values. It only works if the BQ75614 is in active mode
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the soft reset was successful
 *
 * @note The BQ75614 is reset using the SOFT_RESET bit in the CONTROL1 register
 */
BQ75614_StatusType BQ75614_SoftReset(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // Verify that we are in active mode
    if (bq75614->state != BQ75614_STATE_ACTIVE)
    {
        return BQ75614_ERROR_BAD_STATE;
    }

    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_CONTROL1, (uint8_t[]){1 << CONTROL1_SOFT_RESET_POS}, 1));
    // We then wait for the BQ75614 to wake up again
    // We wait for at least 1ms
    BQ75614_Delay(2);

    // We must clear some fault related to the soft reset
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_SYS)); // FAULT_SYS[DRST]
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_COMM1)); // FAULT_COMM1[COMMCLR_DET]
    return BQ75614_OK;
}

// FAULT functions

/**
 * @brief Return a bitfield from the summary fault register. This register is a summary of all the faults that can be detected.
 *
 * @param bq75614 The BQ75614 handle
 * @param status A buffer to store the status of the faults. See `Table 8-17` of the datasheet for more infos about the faults
 * @return BQ75614_StatusType BQ75614_OK if the fault status was read successfully
 */
BQ75614_StatusType BQ75614_GetFaultStatus(BQ75614_HandleTypeDef *bq75614, uint8_t *status)
{
    CHECK_NULL(bq75614, status);

    // Read the FAULT_SUMMARY register
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_FAULT_SUMMARY, status, 1));
    return BQ75614_OK;
}

/**
 * @brief Return all fault registers into a struct given as paramater.
 * The first field is the summary fault and user can check its bits to now wich other field it needs to check.
 *
 * @param bq75614 The BQ75614 handle
 * @param status A buffer to store the status of the faults. See `Table 8-17` of the datasheet for more infos about the faults
 */
BQ75614_StatusType BQ75614_GetAllFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_FaultStructType *status)
{
    CHECK_NULL(bq75614, status);

    uint16_t reg[BQ75614_NBR_OF_FAULT_REGISTERS] = {REG_FAULT_SUMMARY, REG_FAULT_COMM1, REG_FAULT_OTP, REG_FAULT_SYS, REG_FAULT_PROT1, REG_FAULT_PROT2, REG_FAULT_OV1, REG_FAULT_OV2,
                                                    REG_FAULT_UV1, REG_FAULT_UV2, REG_FAULT_OT, REG_FAULT_UT, REG_FAULT_COMP_GPIO, REG_FAULT_COMP_VCCB1, REG_FAULT_COMP_VCCB2, REG_FAULT_COMP_VCOW1,
                                                    REG_FAULT_COMP_VCOW2, REG_FAULT_COMP_CBOW1, REG_FAULT_COMP_CBOW2, REG_FAULT_COMP_CBFET1, REG_FAULT_COMP_CBFET2, REG_FAULT_COMP_MISC, REG_FAULT_PWR1,
                                                    REG_FAULT_PWR2, REG_FAULT_PWR3};
    // Read from the FAULT_SUMMARY register to the FAULT_PWR3 register
    for (int i = 0; i < BQ75614_NBR_OF_FAULT_REGISTERS; i++)
    {
        CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, reg[i], ((uint8_t *)status) + i, 1));
    }
    return BQ75614_OK;
}

/**
 * @brief Return the 8 status bits of a specific fault. Faults must be of type `BQ75614_FaultType`
 * See `Table 8-17` of datasheet for more infos about the faults
 *
 * @note This function is to check the lower level fault status.
 * It can also retrieve the status of the summary fault register but use the `BQ75614_GetFaultStatus` function instead.
 * @note Be aware that the faults you want to check are not masked. Use `BQ75614_MaskFault` to mask/unmask a fault
 *
 * @param bq75614 The BQ75614 handle
 * @param fault_to_check The fault type or register address to check
 * @param status A buffer to store the status of the fault
 * @return BQ75614_StatusType BQ75614_OK if the fault status was read successfully
 */
BQ75614_StatusType BQ75614_GetFaultLowerLevelStatus(BQ75614_HandleTypeDef *bq75614, BQ75614_FaultType fault_to_check, uint8_t *status)
{
    CHECK_NULL(bq75614, status);

    // Read the register to which we want to check the fault
    // The BQ75614_FaultType is a list of all the faults that can be checked and are directly related to their address in the register map
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, (uint8_t)fault_to_check, status, 1));
    return BQ75614_OK;
}

/**
 * @brief This function will clear a fault. This will clear the fault in the `FAULT_RST1` or `FAULT_RST2` registers.
 * The fault must be of type `BQ75614_MaskFaultType`
 *
 * @param bq75614 The BQ75614 handle
 * @param fault_to_clear The fault to clear. Must be of type `BQ75614_MaskFaultType`
 * @return BQ75614_StatusType BQ75614_OK if the fault was cleared successfully
 */
BQ75614_StatusType BQ75614_ClearFault(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType fault_to_clear)
{
    // We must clear the fault in the lower register with FAULT_RST1 and FAULT_RST2. They are formatted as the FAULT_MSK registers
    // See BQ75614_MaskFault for more information
    // If the error is still there, the bit will not clear, so maybe we havo to check the fault status again

    CHECK_NULL(bq75614);

    // We start to check if the fault is in the first register or the second
    uint8_t mask_register = 0;
    uint8_t mask_bit = 0;
    if (fault_to_clear < MASK_COMM1)
    {
        mask_register = (uint8_t)REG_FAULT_RST1;
        mask_bit = fault_to_clear;
    }
    else
    {
        mask_register = (uint8_t)REG_FAULT_RST2;
        mask_bit = fault_to_clear - 8;
    }
    // We then clear the bit in the register
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, mask_register, &mask_bit, 1));
    // TODO : Read fault register to see if there is still a fault

    return BQ75614_OK;
}

/**
 * @brief This function will clear all faults. This will clear the faults in the `FAULT_RST1` or `FAULT_RST2` registers.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the faults were cleared successfully
 */
BQ75614_StatusType BQ75614_ClearAllFaults(BQ75614_HandleTypeDef *bq75614)
{
    // We must clear all the faults in the lower register with FAULT_RST1 and FAULT_RST2. They are formatted as the FAULT_MSK registers
    // See BQ75614_MaskFault for more information
    // If the error is still there, the bit will not clear, so maybe we have to check the fault status again

    CHECK_NULL(bq75614);

    // We clear all bits in registers
    uint8_t faults_to_clear[2] = {0xff, 0xff};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_FAULT_RST1, faults_to_clear, 2));

    // TODO : Read fault register to see if there is still a fault

    return BQ75614_OK;
}

/**
 * @brief This function will mask a fault in the correct register.
 *
 * @param bq75614 The BQ75614 handle
 * @param fault The fault to mask. Must be of type `BQ75614_MaskFaultType`
 * @param value_to_set The value to set. 1 to mask the fault, 0 to unmask the fault
 * @return BQ75614_StatusType BQ75614_OK if the fault was masked successfully
 *
 * @note See function implementation for more information about the faults that can be masked
 */
static BQ75614_StatusType BQ75614_SetMask(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType faults, uint8_t new_value)
{
    // +-------------+--------------------+---------------------------------+-----------------------------------------------+
    // | Register    | Masking Bit Name   | Related Low-level Register(s)   | FAULT_SUMMARY Register Bit That               |
    // |             |                    | Affected                        | Will Be Masked                                |
    // +-------------+--------------------+---------------------------------+-----------------------------------------------+
    // | FAULT_MSK1  | [MSK_PROT]         | FAULT_PROT*                     | [FAULT_PROT]                                  |
    // +             +--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_UT]           | FAULT_UT                        | [FAULT_OTUT]                                  |
    // |             | [MSK_OT]           | FAULT_OT                        |                                               |
    // +             +--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_UV]           | FAULT_UV*                       | [FAULT_OVUV]                                  |
    // |             | [MSK_OV]           | FAULT_OV*                       |                                               |
    // +             +--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_COMP]         | FAULT_COMP_*                    | [FAULT_COMP]                                  |
    // +             +--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_SYS]          | FAULT_SYS                       | [FAULT_SYS]                                   |
    // +             +--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_PWR]          | FAULT_PWR*                      | [FAULT_PWR]                                   |
    // +-------------+--------------------+---------------------------------+-----------------------------------------------+
    // | FAULT_MSK2  | [MSK_OTP_CRC]      | FAULT_OTP[CUST_CRC][FACT_CRC]   | [FAULT_OTP]                                   |
    // |             | [MSK_OTP_DATA]     | All non-CRC bits in FAULT_OTP,  |                                               |
    // |             |                    | DEBUG_OTP_*                     |                                               |
    // +-------------+--------------------+---------------------------------+-----------------------------------------------+
    // |             | [MSK_COMM1]        | FAULT_COMM1, DEBUG_UART_*       | [FAULT_COMM1]                                 |
    // +-------------+--------------------+---------------------------------+-----------------------------------------------+
    // set the mask to FAULT_MSK1 or FAULT_MSK2 to impeach the fault to reach FAULT_SUMMARY and trigger the NFAULT pin
    // We start to check if the fault is in the first register or the second
    
    CHECK_NULL(bq75614);

    if (faults == 0)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    uint8_t faults_mask[2] = {0}; // faults_mask[0] = FAULT_MSK1, faults_mask[1] = FAULT_MSK2
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_FAULT_MSK1, faults_mask, 2));
    if (new_value) // we set all bits
    {
        faults_mask[0] |= (faults & 0xff);        // faults_mask[0] = FAULT_MSK1
        faults_mask[1] |= ((faults >> 8) & 0xff); // faults_mask[1] = FAULT_MSK2
    }
    else // we clear all bits
    {
        faults_mask[0] &= ~(faults & 0xff);        // faults_mask[0] = FAULT_MSK1
        faults_mask[1] &= ~((faults >> 8) & 0xff); // faults_mask[1] = FAULT_MSK2
    }
    // We then write the new mask register
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_FAULT_MSK1, faults_mask, 2));
    // Finally we read it again to verify that bit was correctly set
    BQ75614_MaskFaultType faults_read = 0;
    CHECK_BQ75614_ERROR(BQ75614_GetMaskedFaults(bq75614, &faults_read));
    // We check that bits we wanted are either set to 1
    if (new_value)
    {
        // Here we test that the bits we wanted to set are correctly set, but we don't care about the other bits
        if ((faults_read & faults) != faults)
        {
            return BQ75614_ERROR_REGISTER_NOT_SET;
        }
    }
    else
    {
        // Here we test that the bits we wanted to clear are correctly cleared, but we don't care about the other bits
        if ((faults_read & faults) != 0)
        {
            return BQ75614_ERROR_REGISTER_NOT_SET;
        }
    }
    return BQ75614_OK;
}

/**
 * @brief This function will mask faults. This will prevent the `NFAULT` pin to be triggered by all theses faults.
 * Please refer to comment in code implementation or `Table 8-19` of the datasheet.
 *
 * @param bq75614 The BQ75614 handle
 * @param faults_to_mask faluts to mask. Must be of type `BQ75614_MaskFaultType` and can be separated by ``|`` operator.
 * Example : `MASK_OT | MASK_UT`
 * @return BQ75614_StatusType BQ75614_OK if faults were masked successfully
 */
BQ75614_StatusType BQ75614_MaskFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType faults_to_mask)
{
    return BQ75614_SetMask(bq75614, faults_to_mask, 1);
}

/**
 * @brief This function will unmask faults. This will allow the `NFAULT` pin to be triggered by all theses faults.
 * Please refer to comment in code implementation or `Table 8-19` of the datasheet.
 *
 * @param bq75614 The BQ75614 handle
 * @param faults_to_unmask faluts to unmask. Must be of type `BQ75614_MaskFaultType` and can be separated by ``|`` operator.
 * Example : `MASK_OT | MASK_UT`
 * @return BQ75614_StatusType BQ75614_OK if faults were unmasked successfully
 */
BQ75614_StatusType BQ75614_UnmaskFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType faults_to_unmask)
{
    return BQ75614_SetMask(bq75614, faults_to_unmask, 0);
}

/**
 * @brief This function will return the masked faults. This will return the faults that are currently masked.
 * Please refer to comment in code implementation or `Table 8-19` of the datasheet.
 *
 * @param bq75614 The BQ75614 handle
 * @param faults_masked A buffer to store the masked faults. Must be of type `BQ75614_MaskFaultType`.
 * User must check each bit he want to test with the `MASK_` macros. Exemple : `if(faults_masked & MASK_OT){}`
 * @return BQ75614_StatusType BQ75614_OK if the masked faults were read successfully
 */
BQ75614_StatusType BQ75614_GetMaskedFaults(BQ75614_HandleTypeDef *bq75614, BQ75614_MaskFaultType *faults_masked)
{
    CHECK_NULL(bq75614, faults_masked);

    uint8_t mask[2] = {0};
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_FAULT_MSK1, mask, 2));

    *faults_masked = mask[0] | (mask[1] << 8);

    return BQ75614_OK;
}

/**
 * @brief Enables the `NFAULT` pin. This pin will be triggered if a fault is detected.
 *
 * @note The NFAULT pin is active low. It will be low if a fault is detected.
 * @note If a fault should be detected but is not, verify if it's not masked with `BQ75614_MaskFault`
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the NFAULT pin was enabled successfully
 */
BQ75614_StatusType BQ75614_EnableFaultPin(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We retrieve the actual value of the register
    uint8_t dev_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_CONF, &dev_conf, 1));
    // We set the NFAULT_EN bit to 1
    dev_conf |= (1 << NFAULT_EN_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_DEV_CONF, &dev_conf, 1));
    // Verify that the bit was correctly set
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_CONF, &dev_conf, 1));
    if ((dev_conf & (1 << NFAULT_EN_POS)) == 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }
    return BQ75614_OK;
}

/**
 * @brief Disables the `NFAULT` pin. This pin will not be triggered if a fault is detected.
 *
 * @note The NFAULT pin is active low. It will be high if a fault is detected.
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the NFAULT pin was disabled successfully
 */
BQ75614_StatusType BQ75614_DisableFaultPin(BQ75614_HandleTypeDef *bq75614)
{

    CHECK_NULL(bq75614);

    // We retrieve the actual value of the register
    uint8_t dev_conf = 0;
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_CONF, &dev_conf, 1));
    // We set the NFAULT_EN bit to 0
    dev_conf &= ~(1 << NFAULT_EN_POS);
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceWrite(bq75614, REG_DEV_CONF, &dev_conf, 1));
    // Verify that the bit was correctly set
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_DEV_CONF, &dev_conf, 1));
    if ((dev_conf & (1 << NFAULT_EN_POS)) != 0)
    {
        return BQ75614_ERROR_REGISTER_NOT_SET;
    }
    return BQ75614_OK;
}

// UART functions

/**
 * @brief Send a frame to the BQ75614
 *
 * @param frame The frame to send
 * @param frame_size The size of the frame
 * @return BQ75614_StatusType BQ75614_OK if the frame was sent successfully
 */
BQ75614_StatusType BQ75614_SingleDeviceRead(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    return BQ75614_ReceiveFrame(bq75614, SINGLE_DEVICE_READ, reg, data_buf, data_buf_size);
}

/**
 * @brief Send a frame to the BQ75614 to write a register
 *
 * @param bq75614 The BQ75614 handle
 * @param reg The register to write to
 * @param data_buf The data to write to the register
 * @param data_buf_size The size of the data to write
 * @return BQ75614_StatusType
 */
BQ75614_StatusType BQ75614_SingleDeviceWrite(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    return BQ75614_SendFrame(bq75614, SINGLE_DEVICE_WRITE, reg, data_buf, data_buf_size);
}

/**
 * @brief Send a broadcast frame to write a register
 * This function will write to all devices on the bus
 *
 * @param bq75614 The BQ75614 handle
 * @param reg The register to write to
 * @param data_buf The data to write to the register
 * @param data_buf_size The size of the data to write
 * @return BQ75614_StatusType BQ75614_OK if the frame was sent successfully
 */
BQ75614_StatusType BQ75614_BroadcastWrite(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    return BQ75614_SendFrame(bq75614, BROADCAST_WRITE, reg, data_buf, data_buf_size);
}

/**
 * @brief Send a broadcast frame to read a register
 * This function will read from all devices on the bus
 *
 * @param bq75614 The BQ75614 handle
 * @param reg The register to read from
 * @param data_buf The buffer to store the data read
 * @param data_buf_size The size of the buffer
 * @return BQ75614_StatusType BQ75614_OK if the frame was sent successfully
 */
BQ75614_StatusType BQ75614_BroadcastRead(BQ75614_HandleTypeDef *bq75614, uint16_t reg, uint8_t *data_buf, uint8_t data_buf_size)
{
    return BQ75614_ReceiveFrame(bq75614, BROADCAST_READ, reg, data_buf, data_buf_size);
}

/**
 * @brief Send a ping of duration `ping` to the `BQ75614`
 *
 * @param bq75614 The BQ75614 handle
 * @param ping The ping to send. Must be of type `BQ75614_PingType`
 * @return BQ75614_StatusType BQ75614_OK if the ping was successful
 */
BQ75614_StatusType BQ75614_Ping(BQ75614_HandleTypeDef *bq75614, BQ75614_PingType ping)
{
    // Set the PING pin (TX of UART HOST) to low for a specific time.
    //                          min   | max
    // Time SLEEP_TO_ACTIVE     250us | 350us -> equals a COMM CLEAR which is to clear the UART engine
    // Time WAKE                2ms   | 2.5ms
    // Time SHUTDOWN            7ms   | 10ms
    // Time HW_RESET           36ms   | NaN ms
    CHECK_NULL(bq75614);

    if (ping < PING_SLEEP_TO_ACTIVE || ping > PING_HW_RESET)
    {
        return BQ75614_ERROR_BAD_PARAMETER;
    }

    CHECK_BQ75614_ERROR(BQ75614_PingSetLowTime(bq75614, ping));
    return BQ75614_OK;
}

/**
 * @brief This function send a ping of type `WAKEUP` to the `BQ75614`.
 * It will wake up the `BQ75614` from sleep or shutdown mode.
 * Be aware that this will also apply a `soft reset` and the device should be reconfigured.
 * The ping consists of putting the RX pin of the UART to 0 for minimum time of `2ms` and a maximum of `2.5ms`.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the wakeup was successful
 */
BQ75614_StatusType BQ75614_WakeUpPing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We send a WAKEUP ping to the BQ75614
    // We put the RX pin to 0 for 2ms
    CHECK_BQ75614_ERROR(BQ75614_Ping(bq75614, PING_WAKE));
    // We then wait for the BQ75614 to wake up
    // We wait for 10ms
    BQ75614_Delay(15);
    // Todo : check if device is really up

    // We must clear some fault related to the soft reset
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_SYS)); // FAULT_SYS[DRST]
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_COMM1)); // FAULT_COMM1[COMMCLR_DET]
    bq75614->state = BQ75614_STATE_ACTIVE;
    return BQ75614_OK;
}

/**
 * @brief This function send a ping of type `SLEEP_to_ACTIVE` to the `BQ75614`.
 * If the `BQ75614` is sleeping, it will put it in ACTIVE mode without doing a soft reset.
 * The ping consists of putting the RX pin of the UART to 0 for minimum time of `250us` and a maximum of `350us`.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the command was successful
 */
BQ75614_StatusType BQ75614_SleepToActivePing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We send a SLEEP_to_ACTIVE ping to the BQ75614
    // We put the RX pin to 0 for 2ms
    CHECK_BQ75614_ERROR(BQ75614_Ping(bq75614, PING_SLEEP_TO_ACTIVE));
    // We then wait for the BQ75614 to wake up
    // We wait for 10ms
    BQ75614_Delay(10);
    // Todo : check if device is really up

    // We must clear some fault related to the soft reset
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_COMM1)); // FAULT_COMM1[COMMCLR_DET]
    bq75614->state = BQ75614_STATE_ACTIVE;
    return BQ75614_OK;
}

/**
 * @brief This function send a ping of type `SHUTDOWN` to the `BQ75614`.
 * If the `BQ75614` is active, it will put it in SHUTDOWN.
 * Only a `WAKEUP` ping or a `HARD RESET` can wake up the `BQ75614` from SHUTDOWN mode but all registers will be reset.
 * The ping consists of putting the RX pin of the UART to 0 for minimum time of `7ms` and a maximum of `10ms`.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the command was successful
 */
BQ75614_StatusType BQ75614_ShutdownPing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    // We send a SHUTDOWN ping to the BQ75614
    // We put the RX pin to 0 for 7ms
    CHECK_BQ75614_ERROR(BQ75614_Ping(bq75614, PING_SHUTDOWN));
    bq75614->state = BQ75614_STATE_SHUTDOWN;
    return BQ75614_OK;
}

/**
 * @brief This function send a ping of type `HARDWARE_RESET` to the `BQ75614`.
 * If the `BQ75614` is ACTIVE or in SLEEP_MODE, it will proceed to a hardware reset.
 * The ping consists of putting the RX pin of the UART to 0 for minimum time of `36ms` and there is no maximum.
 *
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the command was successful
 */
BQ75614_StatusType BQ75614_HardResetPing(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // We send a HARDWARE_RESET ping to the BQ75614
    // We put the RX pin to 0 for 36ms
    CHECK_BQ75614_ERROR(BQ75614_Ping(bq75614, PING_HW_RESET));
    // We then wait for the BQ75614 to wake up
    // We wait at least 75ms -> cf. chap. 7.6 of datasheet
    BQ75614_Delay(75);
    // Todo : check if device is really up

    // We must clear some fault related to the soft reset
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_SYS)); // FAULT_SYS[DRST]
    CHECK_BQ75614_ERROR(BQ75614_ClearFault(bq75614, MASK_COMM1)); // FAULT_COMM1[COMMCLR_DET]
    return BQ75614_OK;
}

// OTP Functions

/**
 * @brief This function will program the values set in the OTP registers to the OTP memory of the `BQ75614`.
 * Values that will be registered to Non Volatile Memory need to be written to register first.
 *
 *
 * @note 8.3.6.3.2 OTP Programming of datasheet for more information
 * @param bq75614 The BQ75614 handle
 * @return BQ75614_StatusType BQ75614_OK if the OTP was programmed successfully
 */
BQ75614_StatusType BQ75614_ProgramOTP(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    BQ75614_StatusType err;
    // We must check that OTP_CUST*_STAT[TRY] = 0 and OTP_CUST*_STAT[FMTERR] = 0
    uint8_t data[2];
    err = BQ75614_SingleDeviceRead(bq75614, REG_OTP_CUST1_STAT, data, 2); // read both OTP_CUST1_STAT and OTP_CUST2_STAT
    if (err != BQ75614_OK)
    {
        return err;
    }

    // Check status for page 2
    if ((data[1] & (1 << OTP_CUST_STAT_TRY_POS)) || (data[1] & (1 << OTP_CUST_STAT_FMTERR_POS)))
    {
        return BQ75614_ERROR_OTP_NOT_READY;
    }
    // unlock pages to be written
    uint8_t data_for_unlock1[4] = {0x02, 0xB7, 0x78, 0xBC};
    uint8_t data_for_unlock2[4] = {0x7E, 0x12, 0x08, 0x6F};
    err = BQ75614_SingleDeviceWrite(bq75614, REG_OTP_PROG_UNLOCK1A, data_for_unlock1, 4);
    err = BQ75614_SingleDeviceWrite(bq75614, REG_OTP_PROG_UNLOCK2A, data_for_unlock2, 4);

    // We must check that the unlock was successful
    err = BQ75614_SingleDeviceRead(bq75614, REG_OTP_PROG_STAT, data, 1); // read Programation status
    if (err != BQ75614_OK)
    {
        return err;
    }

    if ((data[0] & (1 << OTP_PROG_STAT_UNLOCK_POS)) == 0)
    {
        return BQ75614_ERROR_OTP_UNLOCK_FAILED;
    }

    // We can now start programming on page 2 (because it has greater priority)
    data[0] = (1 << OTP_PROG_CTRL_PAGESEL_POS) | (1 << OTP_PROG_CTRL_PROG_GO_POS);
    err = BQ75614_SingleDeviceWrite(bq75614, REG_OTP_PROG_CTRL, data, 1);

    BQ75614_Delay(OTP_PROG_TIME); // wait for the OTP to be programmed

    // Check for DONE bit in OTP_PROG_STAT
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTP_PROG_STAT, data, 1));

    if (data[0] & (1 << OTP_PROG_STAT_DONE_POS))
    {
        return BQ75614_ERROR_OTP_PROGRAMMING_FAILED;
    }

    // Check that LOADED, PROGOK, TRY, OVOK and UVOK are all 1 in OTP_CUST2_STAT
    // This check is for Page 2. Read documentation for page 1
    CHECK_BQ75614_ERROR(BQ75614_SingleDeviceRead(bq75614, REG_OTP_CUST2_STAT, data, 1));
    uint8_t prog_ok = (1 << OTP_CUST_STAT_LOADED_POS) | (1 << OTP_CUST_STAT_PROGOK_POS) | (1 << OTP_CUST_STAT_TRY_POS) |
                      (1 << OTP_CUST_STAT_OVOK_POS) | (1 << OTP_CUST_STAT_UVOK_POS);

    if ((data[0] & prog_ok) != prog_ok)
    {
        return BQ75614_ERROR_OTP_PROGRAMMING_FAILED;
    }

    // We can now issue a soft reset to apply the new OTP values
    CHECK_BQ75614_ERROR(BQ75614_SoftReset(bq75614));

    return BQ75614_OK;
}

// Diagnostic functions

BQ75614_StatusType BQ75614_StartPowerBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);

    // This function will test the power supply and the fault generated.
    // It will apply bad voltage to a power supply, check if the fault is generated and then clear it
    // Each fault will also trigger the NFAULT pin if enabled
    // Here we must ensure that
    // - Ensure there is no fault before starting the test
    // - We enable the NFAULT pin to see if it is triggered
    // - We enable TSREF to test it also. Then we put it to it's normal state
    // - Mask all non puwer supply faults. Then we put them back to normal
    // We can then start the BIST (Built In Self Test) with DIAG_PWR_CTRL[PWR_BIST_GO] = 1
    // At the end we check for FAULT_PWR2[PWRBIST_FAIL]
    // If there is a failure, we can restart the test but with DIAG_PWR_CTRL[BIST_NO_RST] = 1
    // Now at the end of the test we can examine FAULT_PWR1 and FAULT_PWR2. If any flag is 0, it means it can't flag a failure
    return BQ75614_ERROR;
}
BQ75614_StatusType BQ75614_StartOVUVBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    
    // see chapter 8.3.6.4.5.3 OVUV Protector BIST of datasheet
    return BQ75614_ERROR;
}
BQ75614_StatusType BQ75614_StartOTUTBuiltInSelfTest(BQ75614_HandleTypeDef *bq75614)
{
    CHECK_NULL(bq75614);
    
    // see chapter 8.3.6.4.5.4 OTUT Protector BIST of datasheet
    return BQ75614_ERROR;
}
