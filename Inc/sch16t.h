/******************************************************************************************* */
/*                                                                                           */
/* Murata SCH16T-K01 library                                                                 */
/*                                                                                           */
/* This library provides functions to interface an STM32H7 microcontroller with the          */
/* Murata SCH1600 sensor through SPI.                                                          */
/*                                                                                           */
/* Florian TOPEZA & Merlin KOOSHMANIAN - 2025                                                */
/*                                                                                           */
/******************************************************************************************* */

#ifndef MURATA_H
// Header guard to prevent multiple inclusions
#define MURATA_H

/******************************* INCLUDES BEGIN ******************************************** */

/** Include standard libraries */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

/** Include STM32 HAL */
#include "stm32h7xx_hal.h"

/* Include error enum type */
#include "errors.h"

/******************************* INCLUDES END ********************************************** */

/******************************* DEFINE BEGIN ********************************************** */

/* Debug MPR */
#ifdef DEBUG_MURATA
#include "console.h"
#define logs_murata(...) printf(__VA_ARGS__)
#else
#define logs_murata(...)
#endif

// Low Pass Filters Settings (3 bits)   Nominal digital cut-off frequency (-3dB)
#define LPF0 0x00u // 0b000, 68 Hz (default)
#define LPF1 0x01u // 0b001, 30 Hz
#define LPF2 0x02u // 0b010, 13 Hz
#define LPF3 0x03u // 0b011, 280 Hz
#define LPF4 0x04u // 0b100, 370 Hz
#define LPF5 0x05u // 0b101, 235 Hz
#define LPF6 0x06u // 0b110, Reserved
#define LPF7 0x07u // 0b111, Bypass

// Dynamic range settings (3 bits) (for CTRL_RATE, CTRL_ACC12 and CTRL_ACC3 registers)
#define UNDEFINED 0x00u // 0b000, Undefined
#define DYN0 0x00u      // 0b000
#define DYN1 0x01u      // 0b001, Default
#define DYN2 0x02u      // 0b010
#define DYN3 0x03u      // 0b100
#define DYN4 0x04u      // 0b101

// Decimation ratio settings (3 bits) (for CTRL_RATE and CTRL_ACC12 registers)
#define DEC1 0x00u // 0b000, No decimation, reduction factor 1, nominal F_PRIM=11,8kHz
#define DEC2 0x01u // 0b001, Reduction factor 2, nominal F_PRIM=5,9kHz
#define DEC3 0x02u // 0b010, Reduction factor 4, nominal F_PRIM=2,95kHz
#define DEC4 0x03u // 0b011, Reduction factor 8, nominal F_PRIM=1,475kHz
#define DEC5 0x04u // 0b100, Reduction factor 16, nominal F_PRIM=738,5Hz

// Sensor control block registers (read/write)
#define CTRL_FILT_RATE 0x0025u  // RATE_XYZ Filter settings
#define CTRL_FILT_ACC12 0x0026u // ACC Filter setting
#define CTRL_FILT_ACC3 0x0027u  // Filter setting for ACC_(X,Y,Z)3

#define CTRL_RATE 0x0028u  // Settings for Gyro post-processing decimation ratio and dynamic range
#define CTRL_ACC12 0x0029u // Settings for ACC_(X,Y,Z)12 post-processing decimation and dynamic range
#define CTRL_ACC3 0x002Au  // Settings for ACC_(X,Y,Z)3 post-processing shift dynamic range

#define CTRL_MODE 0x0035u  // Test mode, EOI, EN_SENSOR
#define CTRL_RESET 0x0036u // SPI soft reset command

// Sensor data block registers (read-only, 16-bit or 20-bit data depending on the SPI frame type)
#define RATE_X1 0x0001u // Output, x-axis gyroscope, interpolation, common LPF with RATE_X2
#define RATE_Y1 0x0002u // Output, y-axis gyroscope, interpolation, common LPF with RATE_Y2
#define RATE_Z1 0x0003u // Output, z-axis gyroscope, interpolation, common LPF with RATE_Z2
#define RATE_X2 0x000Au // Output, x-axis gyroscope, configurable decimation filter, common LPF with RATE_X1
#define RATE_Y2 0x000Bu // Output, y-axis gyroscope, configurable decimation filter, common LPF with RATE_Y1
#define RATE_Z2 0x000Cu // Output, z-axis gyroscope, configurable decimation filter, common LPF with RATE_Z1

#define ACC_X1 0x0004u // Output, x-axis accelerometer, interpolation, common LPF with ACC_X2
#define ACC_Y1 0x0005u // Output, y-axis accelerometer, interpolation, common LPF with ACC_Y2
#define ACC_Z1 0x0006u // Output, z-axis accelerometer, interpolation, common LPF with ACC_Z2
#define ACC_X2 0x000Du // Output, x-axis accelerometer, configurable decimation filter, common LPF with ACC_X1
#define ACC_Y2 0x000Eu // Output, y-axis accelerometer, configurable decimation filter, common LPF with ACC_Y1
#define ACC_Z2 0x000Fu // Output, z-axis accelerometer, configurable decimation filter, common LPF with ACC_Z1
#define ACC_X3 0x0007u // Output, x-axis accelerometer, auxiliary signal path with interpolation and individually configurable LPF settings
#define ACC_Y3 0x0008u // Output, y-axis accelerometer, auxiliary signal path with interpolation and individually configurable LPF settings
#define ACC_Z3 0x0009u // Output, z-axis accelerometer, auxiliary signal path with interpolation and individually configurable LPF settings

#define TEMP 0x0010u // Output, temperature sensor

// Sensor status registers (read-only 16-bit data otherwise noted)
#define STAT_SUM 0x0014u         // Status, summary for non saturation related flags
#define STAT_SUM_SAT 0x0015u     // Status, summary saturation flags
#define STAT_COM 0x0016u         // Common status flags, including TEMP, first level status register
#define STAT_RATE_COM 0x0017u    // Common gyro status flags (primary channel), first level status register
#define STAT_RATE_X 0x0018u      // Status, rate x-axis, first level status register
#define STAT_RATE_Y 0x0019u      // Status, rate y-axis, first level status register
#define STAT_RATE_Z 0x001Au      // Status, rate z-axis, first level status register
#define STAT_ACC_X 0x001Bu       // Status, accelerometer x-axis, first level status register
#define STAT_ACC_Y 0x001Cu       // Status, accelerometer y-axis, first level status register
#define STAT_ACC_Z 0x001Du       // Status, accelerometer z-axis, first level status register
#define STAT_SYNC_ACTIVE 0x001Eu // Status of SYNC on each channel, 11-bits
#define STAT_INFO 0x001Fu        // Low power mode indications, 8-bits

// Miscellaneous registers
#define SYS_TEST 0x0037u // 16-bit read/write register which can be used to check accessibility of the device, or if
                         // multiple devices are connected to the SPI bus to check if CS signals are working properly.
                         // Due to off-frame protocol, test sequence should be as follows:
                         // 1. Write data into SYS_TEST register
                         // 2. Read SYS_TEST register content
                         // 3. Issue a dummy read command to receive response from previous frame
                         // SYS_TEST register is not locked by EOI bit.
#define SN_ID1 0x003Du   // Component Serial Number field 1, 4-bit read-only register, contains F part of the serial number, Format: DDDYYHHHHHH01
#define SN_ID2 0x003Eu   // Component Serial Number field 2, 16-bit read-only register, contains DDDYY part of the serial number, Format: DDDYYHHHHHH01
#define SN_ID3 0x003Fu   // Component Serial Number field 3, 16-bit read-only register, contains HHHH part of the serial number, Format: DDDYYHHHHHH01

/* SCH1 Standard requests */

// Rate and acceleration read commands
#define REQ_READ_RATE_X1 0x0048000000AC
#define REQ_READ_RATE_Y1 0x00880000009A
#define REQ_READ_RATE_Z1 0x00C80000006D
#define REQ_READ_ACC_X1 0x0108000000F6
#define REQ_READ_ACC_Y1 0x014800000001
#define REQ_READ_ACC_Z1 0x018800000037
#define REQ_READ_ACC_X3 0x01C8000000C0
#define REQ_READ_ACC_Y3 0x02080000002E
#define REQ_READ_ACC_Z3 0x0248000000D9
#define REQ_READ_RATE_X2 0x0288000000EF
#define REQ_READ_RATE_Y2 0x02C800000018
#define REQ_READ_RATE_Z2 0x030800000083
#define REQ_READ_ACC_X2 0x034800000074
#define REQ_READ_ACC_Y2 0x038800000042
#define REQ_READ_ACC_Z2 0x03C8000000B5

// Status read commands
#define REQ_READ_STAT_SUM 0x05080000001C
#define REQ_READ_STAT_SUM_SAT 0x0548000000EB
#define REQ_READ_STAT_COM 0x0588000000DD
#define REQ_READ_STAT_RATE_COM 0x05C80000002A
#define REQ_READ_STAT_RATE_X 0x0608000000C4
#define REQ_READ_STAT_RATE_Y 0x064800000033
#define REQ_READ_STAT_RATE_Z 0x068800000005
#define REQ_READ_STAT_ACC_X 0x06C8000000F2
#define REQ_READ_STAT_ACC_Y 0x070800000069
#define REQ_READ_STAT_ACC_Z 0x07480000009E

// Temperature and traceability read commands
#define REQ_READ_TEMP 0x0408000000B1
#define REQ_READ_SN_ID1 0x0F4800000065
#define REQ_READ_SN_ID2 0x0F8800000053
#define REQ_READ_SN_ID3 0x0FC8000000A4
#define REQ_READ_COMP_ID 0x0F0800000092

// Filters read commands
#define REQ_READ_FILT_RATE 0x0948000000FA
#define REQ_READ_FILT_ACC12 0x0988000000CC
#define REQ_READ_FILT_ACC3 0x09C80000003B
#define REQ_READ_RATE_CTRL 0x0A08000000D5
#define REQ_READ_ACC12_CTRL 0x0A4800000022
#define REQ_READ_ACC3_CTRL 0x0A8800000014
#define REQ_READ_MODE_CTRL 0x0D4800000010

#define REQ_SET_FILT_RATE 0x0968000000  // For building Rate_XYZ1/2 filter setting frame.
#define REQ_SET_FILT_ACC12 0x09A8000000 // For building Acc_XYZ1/2 filter setting frame.
#define REQ_SET_FILT_ACC3 0x09E8000000  // For building Acc_XYZ3 filter setting frame.

// Sensitivity and decimation
#define REQ_SET_RATE_CTRL 0x0A28000000  // For building Rate_XYZ1/2 sensitivity and
                                        //     Rate_XYZ2 decimation setting frame.
#define REQ_SET_ACC12_CTRL 0x0A68000000 // For building Acc_XYZ1/2 sensitivity and
                                        //     Acc_XYZ2 decimation setting frame.
#define REQ_SET_ACC3_CTRL 0x0AA8000000  // For building Acc_XYZ3 sensitivity setting frame.
#define REQ_SET_MODE_CTRL 0x0D68000000  // For building MODE-register setting frame.

// DRY/SYNC configuration
#define REQ_READ_USER_IF_CTRL 0x0CC80000007C
#define REQ_SET_USER_IF_CTRL 0x0CE8000000 // For building USER_IF_CTRL -register setting frame.

// Other
#define REQ_SOFTRESET 0x0DA800000AC3 // SPI soft reset command.

/* Frame field masks */

#define TA_FIELD_MASK 0xFFC000000000
#define SA_FIELD_MASK 0x7FE000000000
#define DATA_FIELD_MASK 0x00000FFFFF00
#define CRC_FIELD_MASK 0x0000000000FF
#define ERROR_FIELD_MASK 0x001E00000000

/*********************************** DEFINE END ******************************************** */

/******************************** TYPEDEF BEGIN ******************************************** */

// Structure for Filter
typedef struct
{
    uint16_t stat_sum_rx;      // STAT_SUM register data
    uint16_t stat_sum_sat_rx;  // STAT_SUM_SAT register data
    uint16_t stat_com_rx;      // STAT_COM register data
    uint16_t stat_rate_com_rx; // STAT_RATE_COM register data
    uint16_t stat_rate_x_rx;   // STAT_RATE_X register data
    uint16_t stat_rate_y_rx;   // STAT_RATE_Y register data
    uint16_t stat_rate_z_rx;   // STAT_RATE_Z register data
    uint16_t stat_acc_x_rx;    // STAT_ACC_X register data
    uint16_t stat_acc_y_rx;    // STAT_ACC_Y register data
    uint16_t stat_acc_z_rx;    // STAT_ACC_Z register data
} SCH1_Status;

// Structure for Filter
typedef struct
{
    uint8_t Rate12; // RATE_XYZ Low Pass Filter
    uint8_t Acc12;  // ACC Low Pass Filter 1
    uint8_t Acc3;   // ACC Low Pass Filter for ACC_(X,Y,Z)3
} SCH1_Filter_t;

// Structure for Sensitivity
typedef struct
{
    uint8_t Rate1; // Sensitivity for RATE_X1, RATE_Y1, RATE_Z1
    uint8_t Rate2; // Sensitivity for RATE_X2, RATE_Y2, RATE_Z2
    uint8_t Acc1;  // Sensitivity for ACC_X1, ACC_Y1, ACC_Z1
    uint8_t Acc2;  // Sensitivity for ACC_X2, ACC_Y2, ACC_Z2
    uint8_t Acc3;  // Sensitivity for ACC_X3, ACC_Y3, ACC_Z3
} SCH1_Sensitivity_t;

// Structure for Decimation
typedef struct
{
    uint8_t Rate2;
    uint8_t Acc2;
} SCH1_Decimation_t;

// SCH1 rate raw unprocessed data values
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} SCH1_raw_rate_t;

// SCH1 acceleration raw unprocessed data values
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} SCH1_raw_acc_t;

// SCH1 temperature raw unprocessed data values
typedef int32_t SCH1_raw_temp_t;

// SCH1 rate data values
typedef struct
{
    float x;
    float y;
    float z;
} SCH1_rate_t;

// SCH1 acceleration data values
typedef struct
{
    float x;
    float y;
    float z;
} SCH1_acc_t;

// SCH1 temperature data values
typedef float SCH1_temp_t;

// SCH1 scaled measurement results
typedef struct
{
    float Rate1[3];
    float Rate2[3];
    float Acc1[3];
    float Acc2[3];
    float Acc3[3];
    float Temp;
} SCH1_result_t;

// Structure for Murata Sensor instance
typedef struct
{
    SPI_HandleTypeDef *p_hspi; // SPI Typedef instance pointer

    uint32_t cs_pin;            // Chip Select GPIO pin number
    GPIO_TypeDef *cs_gpio_port; // Chip Select GPIO port

    uint32_t reset_pin;            // Reset pin GPIO instance
    GPIO_TypeDef *reset_gpio_port; // reset pin GPIO port

    SCH1_Filter_t filter_settings;
    SCH1_Sensitivity_t sensitivity_settings;
    SCH1_Decimation_t decimation_settings;
} SCH1Sensor_t;

// Serial Number
typedef char SCH1_Snbr_t[11];

/******************************** TYPEDEF END ********************************************** */

/********************************** MACRO BEGIN ******************************************** */

// SPI48BF frame construction
#define SPI48BF_CS_OFFSET 46
#define SPI48BF_CS_MASK 0x02ULL
#define SPI48BF_ADDR_OFFSET 38
#define SPI48BF_ADDR_MASK 0xFFULL
#define SPI48BF_RW_OFFSET 37
#define SPI48BF_RW_MASK 0x01ULL
#define SPI48BF_FT_OFFSET 35
#define SPI48BF_FT_MASK 0x01ULL
#define SPI48BF_DATAI_OFFSET 8
#define SPI48BF_DATAI_MASK 0xFFFFFULL
#define SPI48BF_CRC8_OFFSET 0
#define SPI48BF_CRC8_MASK 0xFFULL

#define SPI_FRAME48BF(CS, ADDR, RW, FT, DATAI, CRC8)                    \
    (((uint64_t)(CS) & SPI48BF_CS_MASK) << SPI48BF_CS_OFFSET |          \
     ((uint64_t)(ADDR) & SPI48BF_ADDR_MASK) << SPI48BF_ADDR_OFFSET |    \
     ((uint64_t)(RW) & SPI48BF_RW_MASK) << SPI48BF_RW_OFFSET |          \
     ((uint64_t)(FT) & SPI48BF_FT_MASK) << SPI48BF_FT_OFFSET |          \
     ((uint64_t)(DATAI) & SPI48BF_DATAI_MASK) << SPI48BF_DATAI_OFFSET | \
     ((uint64_t)(CRC8) & SPI48BF_CRC8_MASK) << SPI48BF_CRC8_OFFSET)

// SPI32BF frame construction
#define SPI32BF_CS_OFFSET 30
#define SPI32BF_CS_MASK 0x02ULL
#define SPI32BF_ADDR_OFFSET 22
#define SPI32BF_ADDR_MASK 0xFFULL
#define SPI32BF_RW_OFFSET 21
#define SPI32BF_RW_MASK 0x01ULL
#define SPI32BF_FT_OFFSET 19
#define SPI32BF_FT_MASK 0x01ULL
#define SPI32BF_DATAI_OFFSET 3
#define SPI32BF_DATAI_MASK 0xFFFFULL
#define SPI32BF_CRC3_OFFSET 0
#define SPI32BF_CRC3_MASK 0x03ULL

#define SPI_FRAME32BF(CS, ADDR, RW, FT, DATAI, CRC3)                    \
    (((uint64_t)(CS) & SPI32BF_CS_MASK) << SPI32BF_CS_OFFSET |          \
     ((uint64_t)(ADDR) & SPI32BF_ADDR_MASK) << SPI32BF_ADDR_OFFSET |    \
     ((uint64_t)(RW) & SPI32BF_RW_MASK) << SPI32BF_RW_OFFSET |          \
     ((uint64_t)(FT) & SPI32BF_FT_MASK) << SPI32BF_FT_OFFSET |          \
     ((uint64_t)(DATAI) & SPI32BF_DATAI_MASK) << SPI32BF_DATAI_OFFSET | \
     ((uint64_t)(CRC3) & SPI32BF_CRC3_MASK) << SPI32BF_CRC3_OFFSET)

// Filters frame construction
#define LPF_X_OFFSET 3
#define LPF_Y_OFFSET 6
#define LPF_Z_OFFSET 9
#define LPF_MASK 0x03ULL

// RATE_XYZ frame construction
#define RATE_XYZ_DATA_FIELD(LPF_X, LPF_Y, LPF_Z)      \
    (((uint64_t)(LPF_X) & LPF_MASK) << LPF_X_OFFSET | \
     ((uint64_t)(LPF_Y) & LPF_MASK) << LPF_Y_OFFSET | \
     ((uint64_t)(LPF_Z) & LPF_MASK) << LPF_Z_OFFSET)

// ACC12 frame construction
#define ACC12_DATA_FIELD(LPF_X, LPF_Y, LPF_Z)             \
    ((uint64_t)(LPF_X) & LPF_MASK) << LPF_X_OFFSET |      \
        ((uint64_t)((LPF_Y) & LPF_MASK) << LPF_Y_OFFSET | \
         ((uint64_t)(LPF_Z) & LPF_MASK) << LPF_Z_OFFSET)

// ACC3 frame construction
#define ACC3_DATA_FIELD(LPF_X, LPF_Y, LPF_Z)          \
    (((uint64_t)(LPF_X) & LPF_MASK) << LPF_X_OFFSET | \
     ((uint64_t)(LPF_Y) & LPF_MASK) << LPF_Y_OFFSET | \
     ((uint64_t)(LPF_Z) & LPF_MASK) << LPF_Z_OFFSET)

// Dynamic range and decimation frame construction
#define DYN_RATE_XYZ1_OFFSET 12
#define DYN_RATE_XYZ2_OFFSET 19
#define DEC_RATE_Z2_OFFSET 6
#define DEC_RATE_Y2_OFFSET 3
#define DEC_RATE_X2_OFFSET 0
#define DYN_RATE_XYZ1_MASK 0x03ULL
#define DYN_RATE_XYZ2_MASK 0x03ULL
#define DEC_RATE_Z2_MASK 0x03ULL
#define DEC_RATE_Y2_MASK 0x03ULL
#define DEC_RATE_X2_MASK 0x03ULL

#define DYN_ACC_XYZ3_OFFSET 0
#define DYN_ACC_XYZ3_MASK 0x03ULL

// CTRL_RATE frame construction
#define CTRL_RATE_DATA_FIELD(DYN_RATE_XYZ1, DYN_RATE_XYZ2, DEC_RATE_X2, DEC_RATE_Y2, DEC_RATE_Z2) \
    (((uint64_t)(DYN_RATE_XYZ1) & DYN_RATE_XYZ1_MASK) << DYN_RATE_XYZ1_OFFSET |                   \
     ((uint64_t)(DYN_RATE_XYZ2) & DYN_RATE_XYZ2_MASK) << DYN_RATE_XYZ2_OFFSET |                   \
     ((uint64_t)(DEC_RATE_X2) & DEC_RATE_X2_MASK) << DEC_RATE_X2_OFFSET |                         \
     ((uint64_t)(DEC_RATE_Y2) & DEC_RATE_Y2_MASK) << DEC_RATE_Y2_OFFSET |                         \
     ((uint64_t)(DEC_RATE_Z2) & DEC_RATE_Z2_MASK) << DEC_RATE_Z2_OFFSET)

// CTRL_ACC12 frame construction
#define CTRL_ACC12_DATA_FIELD(DYN_RATE_XYZ1, DYN_RATE_XYZ2, DEC_RATE_X2, DEC_RATE_Y2, DEC_RATE_Z2) \
    (((uint64_t)(DYN_RATE_XYZ1) & DYN_RATE_XYZ1_MASK) << DYN_RATE_XYZ1_OFFSET |                    \
     ((uint64_t)(DYN_RATE_XYZ2) & DYN_RATE_XYZ2_MASK) << DYN_RATE_XYZ2_OFFSET |                    \
     ((uint64_t)(DEC_RATE_X2) & DEC_RATE_X2_MASK) << DEC_RATE_X2_OFFSET |                          \
     ((uint64_t)(DEC_RATE_Y2) & DEC_RATE_Y2_MASK) << DEC_RATE_Y2_OFFSET |                          \
     ((uint64_t)(DEC_RATE_Z2) & DEC_RATE_Z2_MASK) << DEC_RATE_Z2_OFFSET)

// CTRL_ACC3 frame construction
#define CTRL_ACC3_DATA_FIELD(DYN_ACC_XYZ3) \
    (((uint64_t)(DYN_ACC_XYZ3) & DYN_ACC_XYZ3_MASK) << DYN_ACC_XYZ3_OFFSET)

// Double word byte swap macro
// The sensors receives the data byte per byte, MSB first, starting with the most significant byte of the first word
// and ending with the least significant byte of the sixth word.
// It is necessary to swap the bytes of the double word to match the sensor's expected byte order, and to shift the result
// to the right by 16 bits to align the data correctly for transmission.
#define DOUBLE_WORD_BYTE_SWAP(double_word) \
    (((0xff00000000000000ull & (double_word)) >> 56) |  \
    ((0x00ff000000000000ull & (double_word)) >> 40) |   \
    ((0x0000ff0000000000ull & (double_word)) >> 24) |   \
    ((0x000000ff00000000ull & (double_word)) >> 8) |    \
    ((0x00000000ff000000ull & (double_word)) << 8) |    \
    ((0x0000000000ff0000ull & (double_word)) << 24) |   \
    ((0x000000000000ff00ull & (double_word)) << 40) |   \
    ((0x00000000000000ffull & (double_word)) << 56)) >> 16

// Sensor mode control and soft reset frame construction
#define EOI_OFFSET 1
#define EOI_MASK 0x01ULL
#define EN_SENSOR_OFFSET 0
#define EN_SENSOR_MASK 0x01ULL
#define CTRL_MODE_MASK 0x03ULL

#define CTRL_MODE_DATA_FIELD(EOI, EN_SENSOR)                          \
    ((((uint64_t)(EOI) & EOI_MASK) << EOI_OFFSET |                    \
      ((uint64_t)(EN_SENSOR) & EN_SENSOR_MASK) << EN_SENSOR_OFFSET) & \
     CTRL_MODE_MASK)

#define CTRL_RESET_DATA_FIELD 0x0000Au // Soft reset data field

#define SPI48_DATA_INT32(data_buffer) ((int32_t)(data_buffer[2] << 28 | \
                            data_buffer[3] << 20 | data_buffer[4] << 12) >> 12)

#define SPI48_DATA_UINT32(a) ((uint32_t)(((a) >> 8) & 0x000fffffUL))
#define SPI48_DATA_UINT16(a) ((uint16_t)(((a) >> 8) & 0x0000ffffUL))

/********************************** MACRO END ********************************************** */

/************************** FUNCTION PROTOTYPES BEGIN ************************************** */

uint8_t SCH1_reset(SCH1Sensor_t *sensor);
error_t SCH1_SPI_soft_reset(SCH1Sensor_t *sensor);

error_t SCH1_TransmitReceive(SCH1Sensor_t *sensor, uint64_t msg,
                                uint8_t *rx_buffer, uint16_t buffer_size);  // Function to transmit and receive data over SPI

error_t SCH1_init(SCH1Sensor_t *sensor);                                        // Function to initialize the SCH1600 sensor
error_t SCH1_getSnbr(SCH1Sensor_t *sensor, SCH1_Snbr_t *snbr);              // Function to read the serial number from the sensor
error_t SCH1_read_status(SCH1Sensor_t *sensor, SCH1_Status *status);            // Function to read all status registers of the SCH1600 sensor

uint8_t CRC8(uint64_t SPIframe); // Function to calculate CRC8 for the given SPI frame
uint8_t CRC3(uint32_t SPIframe); // Function to calculate CRC3 for the given SPI frame

error_t SCH1_Read_Acc1_Data(SCH1Sensor_t *sensor, SCH1_raw_acc_t *acc1_raw_data);
error_t SCH1_Read_Rate1_Data(SCH1Sensor_t *sensor, SCH1_raw_rate_t *rate1_raw_data);
error_t SCH1_Read_Temp_Data(SCH1Sensor_t *sensor, SCH1_raw_temp_t *temp_raw_data);

uint8_t SCH1_Convert_Acc_Interpolated_Data(SCH1Sensor_t *sensor, SCH1_raw_acc_t *raw_acc, SCH1_acc_t *acc);
uint8_t SCH1_Convert_Rate_Interpolated_Data(SCH1Sensor_t *sensor, SCH1_raw_rate_t *raw_rate, SCH1_rate_t *rate);
uint8_t SCH1_Convert_Temp_Data(SCH1Sensor_t *sensor, SCH1_raw_temp_t *raw_temp, SCH1_temp_t *temp);

/************************** FUNCTION PROTOTYPES END **************************************** */

#endif // MURATA_H

/********************************** END OF FILE ******************************************** */

