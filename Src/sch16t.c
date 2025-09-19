/******************************************************************************************* */
/*                                                                                           */
/* Murata SCH16T-K01 library                                                                 */
/*                                                                                           */
/*                                                                                           */
/* This library provides functions to interface with the Murata SCH1600 sensor using SPI.    */
/*                                                                                           */
/*                                                                                           */
/* Florian TOPEZA - 2025                                                                     */
/*                                                                                           */
/******************************************************************************************* */

#include "sch16t.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_def.h"

/********************************* FUNCTIONS BEGIN ***************************************** */

/* SPI Transmit/Receive Function */
error_t SCH1_TransmitReceive(SCH1Sensor_t *sensor, uint64_t msg, uint8_t *rx_buffer, uint16_t buffer_size)
{

	error_t result = OK;
	HAL_StatusTypeDef HAL_result = HAL_OK;

	HAL_GPIO_WritePin(sensor->cs_gpio_port, sensor->cs_pin, GPIO_PIN_RESET);

	// HAL_SPI_TransmitReceive expects a pointer to the SPI_HandleTypeDef structure
	// p_hspi is the SPI_HandleTypeDef structure pointer defined in the SCH1Sensor_t structure

	HAL_result = HAL_SPI_TransmitReceive(sensor->p_hspi, (uint8_t *)&msg, rx_buffer, buffer_size, HAL_MAX_DELAY);
	if (HAL_result == HAL_ERROR || HAL_result == HAL_TIMEOUT)
	{
		result = ERROR_FROM_HAL;
	}

	HAL_GPIO_WritePin(sensor->cs_gpio_port, sensor->cs_pin, GPIO_PIN_SET);

	return result;
}

/* CRC8 Calculation */
uint8_t CRC8(uint64_t SPIframe) // Calculates crc for 48-bit frame
{
	uint64_t data = SPIframe & 0xFFFFFFFFFF00LL;
	uint8_t crc = 0xFF;
	for (int i = 47; i >= 0; i--)
	{
		uint8_t data_bit = (data >> i) & 0x01;
		crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
	}
	return crc;
}

/* CRC3 Calculation */
uint8_t CRC3(uint32_t SPIframe) // Calculates crc for 32-bit frame
{
	uint32_t data = SPIframe & 0xFFFFFFF8;
	uint8_t crc = 0x05;
	for (int i = 31; i >= 0; i--)
	{
		uint8_t data_bit = (data >> i) & 0x01;
		crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
		crc &= 0x7;
	}
	return crc;
}

/* Get the sensor serial number */
error_t SCH1_getSnbr(SCH1Sensor_t *sensor, SCH1_Snbr_t *snbr)
{

	error_t result = OK;

	// Prepare the SPI frames to read the serial number
	uint8_t CS = 0x00; // Chip Select

	uint8_t ADDR1 = SN_ID1; // Address for serial number
	uint8_t ADDR2 = SN_ID2; // Address for serial number
	uint8_t ADDR3 = SN_ID3; // Address for serial number
	uint8_t ADDR4 = 0x00;	// Not used, but required for the frame structure

	uint8_t RW = 0u; // Read operation
	uint8_t FT = 1u; // Frame type
	uint64_t DATAI = 0ULL;

	uint8_t CRC8_1 = CRC8(SPI_FRAME48BF(CS, ADDR1, RW, FT, DATAI, 0)); // Calculate CRC8 for the first frame
	uint8_t CRC8_2 = CRC8(SPI_FRAME48BF(CS, ADDR2, RW, FT, DATAI, 0)); // Calculate CRC8 for the second frame
	uint8_t CRC8_3 = CRC8(SPI_FRAME48BF(CS, ADDR3, RW, FT, DATAI, 0)); // Calculate CRC8 for the third frame
	uint8_t CRC8_4 = CRC8(SPI_FRAME48BF(CS, ADDR4, RW, FT, DATAI, 0)); // Calculate CRC8 for the fourth frame

	int buffer_size = 6;		// Size of the buffer for each SPI transaction
	uint8_t rx_buffer[6] = {0}; // Buffer to receive data, 6 bytes for each transaction

	uint16_t sn_id1 = 0u;
	uint16_t sn_id2 = 0u;
	uint16_t sn_id3 = 0u;

	// First SPI transaction
	uint64_t msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, ADDR2, RW, FT, DATAI, CRC8_2));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Second SPI transaction, to receive the first part of the serial number and to ask for the second part
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, ADDR1, RW, FT, DATAI, CRC8_1));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	sn_id2 = rx_buffer[4] | (rx_buffer[3] << 8);

	// Third SPI transaction, to receive the second part of the serial number and to ask for the third part
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, ADDR3, RW, FT, DATAI, CRC8_3));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	sn_id1 = rx_buffer[4] & 0x0F;

	// Fourth SPI transaction, to receive the third part of the serial number
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, ADDR4, RW, FT, DATAI, CRC8_4));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	sn_id3 = rx_buffer[4] | (rx_buffer[3] << 8);

	// Convert received data to serial number format
	// Serial number format: DDDYYFHHHHH01
	// DDDYY - Date code, F has no meaning, HHHH - Running number, H01 - Fixed part
	// Date code is obtained from rx_data2, through a 16-bit unsigned integer to decimal string conversion
	// F is obtained from rx_data1, through a 4-bit hex to string conversion
	// HHHH is obtained from rx_data3, through 16-bit hexadecimal running number conversion
	snprintf((char *)snbr, sizeof(SCH1_Snbr_t) + 1, "%05d%01x%04x", sn_id2, sn_id1, sn_id3);


	logs_murata("Serial number:\n\r");
	// Date code extraction
	logs_murata("  SN_ID2 : %05d\n\r", sn_id2); // Convert to decimal string

	// F part extraction
	logs_murata("  SN_ID1 : %01X\n\r", sn_id1); // Convert to hexadecimal string

	// HHHH part extraction
	logs_murata("  SN_ID3 : %04X\n\r", sn_id3); // Convert to hexadecimal string

	return result;
}

/* Reset the sensor */
uint8_t SCH1_reset(SCH1Sensor_t *sensor)
{

	HAL_GPIO_WritePin(sensor->reset_gpio_port, sensor->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(sensor->reset_gpio_port, sensor->reset_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	return 0;
}

/* Soft reset the sensor through SPI */
error_t SCH1_SPI_soft_reset(SCH1Sensor_t *sensor)
{

	error_t result = OK;

	uint16_t buffer_size = 6;		// Size of the buffer for each SPI transaction
	uint8_t rx_buffer[6] = {0}; // Buffer to receive data, 6 bytes for each transaction

	uint8_t CS = 0x00; // Chip Select
	uint8_t RW = 1u;   // Write
	uint8_t FT = 1u;   // 48-bit SPI frame
	uint64_t DATAI = 0x0000AULL;
	uint8_t crc = 0u;

	// Soft reset command
	crc = CRC8(SPI_FRAME48BF(CS, CTRL_RESET, RW, FT, DATAI, 0));
	uint64_t msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, CTRL_RESET, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	HAL_Delay(2); // No communication during 2ms after reset

	return result;
}

/* Start-up sequence */
error_t SCH1_init(SCH1Sensor_t *sensor)
{
	error_t result = OK;

	uint8_t rx_buffer[6] = {0}; // Buffer to receive data, 6 bytes for each transaction
	SCH1_Status status = {0};
	uint64_t msg;

	SCH1_reset(sensor);

	// Wait 33ms for the sensor for NVM read and SPI startup
	HAL_Delay(32);

	// Use default settings, directly write EN_SENSOR=1
	msg = DOUBLE_WORD_BYTE_SWAP(0x0D68000001D3ULL); // Directly use the appropriate command

	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, 6);

	// Wait 215ms
	HAL_Delay(215);

	// Read all status registers once
	result = SCH1_read_status(sensor, &status);

	// Write EOI=1
	msg = DOUBLE_WORD_BYTE_SWAP(0x0D680000038DULL); // Directly use the appropriate command

	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, 6);

	// Wait 3ms
	HAL_Delay(3);

	// Read all status registers twice
	result = SCH1_read_status(sensor, &status);
	result = SCH1_read_status(sensor, &status);

	return result;
}

/* Read all status registers of the sensor */
error_t SCH1_read_status(SCH1Sensor_t *sensor, SCH1_Status *status)
{
	error_t result = OK;

	// Read all status registers once

	uint8_t CS = 0x00; // Chip Select
	uint8_t RW = 0u;   // Read
	uint8_t FT = 1u;   // 48-bit SPI frame
	uint64_t DATAI = 0ULL;
	uint8_t crc = 0u;

	uint8_t rx_buffer[6] = {0}; // Array to store received data
	uint16_t buffer_size = 6;		// Size of the buffer for each SPI transaction

	// Read STAT_SUM register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_SUM, RW, FT, DATAI, 0));
	uint64_t msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_SUM, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Read STAT_SUM_SAT register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_SUM_SAT, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_SUM_SAT, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_SUM register through serial port if console enabled
	status->stat_sum_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_sum_rx: %x\n\r", stat_sum_rx);

	// Read STAT_COM register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_COM, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_COM, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_SUM_SAT register through serial port if console enabled
	status->stat_sum_sat_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_sum_sat_rx: %x\n\r", stat_sum_sat_rx);

	// Read STAT_RATE_COM register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_RATE_COM, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_RATE_COM, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_COM register through serial port if console enabled
	status->stat_com_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_com_rx: %x\n\r", stat_com_rx);

	// Read STAT_RATE_X register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_RATE_X, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_RATE_X, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_RATE_COM register through serial port if console enabled
	status->stat_rate_com_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_rate_com_rx: %x\n\r", stat_rate_com_rx);

	// Read STAT_RATE_Y register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_RATE_Y, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_RATE_Y, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_RATE_X register through serial port if console enabled
	status->stat_rate_x_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_rate_x_rx: %x\n\r", stat_rate_x_rx);

	// Read STAT_RATE_Z register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_RATE_Z, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_RATE_Z, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_RATE_Y register through serial port if console enabled
	status->stat_rate_y_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_rate_y_rx: %x\n\r", stat_rate_y_rx);

	// Read STAT_ACC_X register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_ACC_X, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_ACC_X, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_RATE_Z register through serial port if console enabled
	status->stat_rate_z_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_rate_z_rx: %x\n\r", stat_rate_z_rx);

	// Read STAT_ACC_Y register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_ACC_Y, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_ACC_Y, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_ACC_X register through serial port if console enabled
	status->stat_acc_x_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_acc_x_rx: %x\n\r", stat_acc_x_rx);

	// Read STAT_ACC_Z register
	crc = CRC8(SPI_FRAME48BF(CS, STAT_ACC_Z, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, STAT_ACC_Z, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_ACC_Y register through serial port if console enabled
	status->stat_acc_y_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_acc_y_rx: %x\n\r", stat_acc_y_rx);

	// Dummy SYS_TEST read command to get STAT_ACC_Z data (due to off-frame protocol)
	crc = CRC8(SPI_FRAME48BF(CS, SYS_TEST, RW, FT, DATAI, 0));
	msg = DOUBLE_WORD_BYTE_SWAP(SPI_FRAME48BF(CS, SYS_TEST, RW, FT, DATAI, crc));
	result = SCH1_TransmitReceive(sensor, msg, rx_buffer, buffer_size);

	// Print received data for STAT_ACC_Z register through serial port if console enabled
	status->stat_acc_z_rx = rx_buffer[4] | (rx_buffer[3] << 8);
	logs_murata("stat_acc_z_rx: %x\n\r", stat_acc_z_rx);

	return result;
}
/* Read interpolated acceleratioin data */
error_t SCH1_Read_Acc1_Data(SCH1Sensor_t *sensor, SCH1_raw_acc_t *acc1_raw_data)
{

	error_t result = OK;
	uint8_t buffer_size = 6; // Size of the buffer for each SPI transaction

	uint8_t acc_x_raw_buffer[6] = {0};
	uint8_t acc_y_raw_buffer[6] = {0};
	uint8_t acc_z_raw_buffer[6] = {0};

	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_ACC_Y1), acc_x_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_ACC_Z1), acc_y_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_ACC_X1), acc_x_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_ACC_Z1), acc_z_raw_buffer, buffer_size); // Read z-axis twice due to off-frame response

	// Parse data to structure
	acc1_raw_data->x = SPI48_DATA_INT32(acc_x_raw_buffer);
	acc1_raw_data->y = SPI48_DATA_INT32(acc_y_raw_buffer);
	acc1_raw_data->z = SPI48_DATA_INT32(acc_z_raw_buffer);

	return result;
}

/* Read interpolated gyroscope data */
error_t SCH1_Read_Rate1_Data(SCH1Sensor_t *sensor, SCH1_raw_rate_t *rate1_raw_data)
{
	error_t result = OK;

	uint8_t buffer_size = 6; // Size of the buffer for each SPI transaction
	uint8_t rate_x_raw_buffer[6] = {0};
	uint8_t rate_y_raw_buffer[6] = {0};
	uint8_t rate_z_raw_buffer[6] = {0};

	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_RATE_X1), rate_x_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_RATE_X1), rate_x_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_RATE_Y1), rate_y_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_RATE_Z1), rate_z_raw_buffer, buffer_size); // Read z-axis twice due to off-frame response

	// Parse data to structure
	rate1_raw_data->x = SPI48_DATA_INT32(rate_x_raw_buffer);
	rate1_raw_data->y = SPI48_DATA_INT32(rate_y_raw_buffer);
	rate1_raw_data->z = SPI48_DATA_INT32(rate_z_raw_buffer);

	return result;
}

error_t SCH1_Read_Temp_Data(SCH1Sensor_t *sensor, SCH1_raw_temp_t *temp_raw_data)
{
	error_t result = OK;

	uint8_t temperature_raw_buffer[6] = {0};
	uint8_t buffer_size = 6;

	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_TEMP), temperature_raw_buffer, buffer_size);
	result = SCH1_TransmitReceive(sensor, DOUBLE_WORD_BYTE_SWAP(REQ_READ_TEMP), temperature_raw_buffer, buffer_size); // Read twice due to off-frame response

	// Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
	*temp_raw_data = SPI48_DATA_INT32(temperature_raw_buffer) >> 4;

	return result;
}

/* Convert interpolated Acc1 and Rate1 data */
uint8_t SCH1_Convert_Rate_Interpolated_Data(SCH1Sensor_t *sensor, SCH1_raw_rate_t *raw_rate, SCH1_rate_t *rate)
{
	float rate_sensitivity = 0.0f;

	// Compute the rate sensitivity
	// As this driver is coded with 48-bit SPI frames, we use the sensitivity for
	// 20-bit data
	switch (sensor->sensitivity_settings.Rate1)
	{
	case DYN1:
		rate_sensitivity = 1600.0f;
		break;
	case DYN2:
		rate_sensitivity = 1600.0f;
		break;
	case DYN3:
		rate_sensitivity = 3200.0f;
		break;
	case DYN4:
		rate_sensitivity = 6400.0f;
		break;
	default:
		return -1;
	}

	// Compute converted gyroscope data
	rate->x = (float)raw_rate->x / rate_sensitivity;
	rate->y = (float)raw_rate->y / rate_sensitivity;
	rate->z = (float)raw_rate->z / rate_sensitivity;

	logs_murata("Interpolated rate results: %.2f %.2f %.2f\n\r", rate->x, rate->y, rate->z);

	return 0;
}

/* Convert interpolated Acc1 and Rate1 data */
uint8_t SCH1_Convert_Acc_Interpolated_Data(SCH1Sensor_t *sensor, SCH1_raw_acc_t *raw_acc, SCH1_acc_t *acc)
{
	float acc_sensitivity = 0.0f;

	// Compute the acceleration sensitivity
	// As this driver is coded with 48-bit SPI frames, we use the sensitivity for
	// 20-bit data
	switch (sensor->sensitivity_settings.Acc1)
	{
	case DYN1:
		acc_sensitivity = 3200.0f;
		break;
	case DYN2:
		acc_sensitivity = 6400.0f;
		break;
	case DYN3:
		acc_sensitivity = 12800.0f;
		break;
	case DYN4:
		acc_sensitivity = 25600.0f;
		break;
	default:
		return -1;
	}

	// Compute converted accelerometer data
	acc->x = (float)raw_acc->x / acc_sensitivity;
	acc->y = (float)raw_acc->y / acc_sensitivity;
	acc->z = (float)raw_acc->z / acc_sensitivity;

	logs_murata("Interpolated acceleration results: %.2f %.2f %.2f\n\r", acc->x, acc->y, acc->z);

	return 0;
}

/* Convert temperature data */

uint8_t SCH1_Convert_Temp_Data(SCH1Sensor_t *sensor, SCH1_raw_temp_t *raw_temp, SCH1_temp_t *temp)
{
	// Temperature ensisitivy is always 100
	*temp = (float)*raw_temp / 100.0f;

	logs_murata("Converted temperature: %.2f\n\r", *temp);

	return 0;
}

/************************************* FUNCTION END **************************************** */

/************************************* END OF FILE ***************************************** */

