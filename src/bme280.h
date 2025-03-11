/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * [BSD-3-Clause license text omitted for brevity]
 *
 * @file    bme280.h
 * @date    2025-03-10
 * @brief   Header file for the BME280 environmental sensor driver in a FreeRTOS-based system.
 *
 * Purpose: Declares data structures, macros, and function prototypes for interfacing with
 *          the BME280 sensor, which measures temperature, pressure, and humidity. This
 *          driver is designed for use in an embedded system with FreeRTOS and Xilinx I2C.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 */

#ifndef BME280_H // Include guard to prevent multiple inclusions
#define BME280_H

/*--------------------------------------------------------------------------*/
/* Header Includes                                                          */
/*--------------------------------------------------------------------------*/
#include <stdint.h> // Provides fixed-width integer types (e.g., uint8_t, int32_t)

#include "FreeRTOS.h" // FreeRTOS kernel definitions (e.g., task handling, timing)
#include "semphr.h"   // Semaphore definitions for FreeRTOS (e.g., SemaphoreHandle_t)
#include "xiic.h"     // Xilinx I2C driver for low-level communication with the BME280

/*--------------------------------------------------------------------------*/
/* General Macro Definitions                                                */
/*--------------------------------------------------------------------------*/
/**
 * @def BME280_I2C_ADDR
 * @brief Default I2C address of the BME280 sensor (0x76 when SDO is low).
 */
#define BME280_I2C_ADDR 0x76

/**
 * @def REG_CHIP_ID
 * @brief Register address for the BME280 chip identification (read-only).
 */
#define REG_CHIP_ID 0xD0

/**
 * @def REG_TEMP_PRESS_CALIB
 * @brief Starting register address for temperature and pressure calibration data.
 */
#define REG_TEMP_PRESS_CALIB 0x88

/**
 * @def REG_HUMIDITY_CALIB
 * @brief Starting register address for humidity calibration data.
 */
#define REG_HUMIDITY_CALIB 0xE1

/**
 * @def REG_CTRL_HUM
 * @brief Register address for humidity oversampling control.
 */
#define REG_CTRL_HUM 0xF2

/**
 * @def REG_CTRL_MEAS
 * @brief Register address for temperature/pressure oversampling and mode control.
 */
#define REG_CTRL_MEAS 0xF4

/**
 * @def REG_CONFIG
 * @brief Register address for standby time and filter coefficient configuration.
 */
#define REG_CONFIG 0xF5

/**
 * @def REG_DATA
 * @brief Starting register address for raw sensor data (pressure, temp, humidity).
 */
#define REG_DATA 0xF7

/**
 * @def BME280_CHIP_ID
 * @brief Expected chip ID value for the BME280 sensor (0x60).
 */
#define BME280_CHIP_ID 0x60

/*--------------------------------------------------------------------------*/
/* Structure Definitions                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @struct bme280_calib_data
 * @brief Structure to hold calibration data read from the BME280 sensor.
 *
 * Description: Contains calibration coefficients for temperature (dig_tX), pressure
 *              (dig_pX), and humidity (dig_hX) compensation, as well as t_fine, an
 *              intermediate value used in compensation calculations.
 */
struct bme280_calib_data
{
    uint16_t dig_t1; // Temperature calibration: unsigned 16-bit coefficient
    int16_t  dig_t2; // Temperature calibration: signed 16-bit coefficient
    int16_t  dig_t3; // Temperature calibration: signed 16-bit coefficient
    uint16_t dig_p1; // Pressure calibration: unsigned 16-bit coefficient
    int16_t  dig_p2; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p3; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p4; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p5; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p6; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p7; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p8; // Pressure calibration: signed 16-bit coefficient
    int16_t  dig_p9; // Pressure calibration: signed 16-bit coefficient
    uint8_t  dig_h1; // Humidity calibration: unsigned 8-bit coefficient
    int16_t  dig_h2; // Humidity calibration: signed 16-bit coefficient
    uint8_t  dig_h3; // Humidity calibration: unsigned 8-bit coefficient
    int16_t  dig_h4; // Humidity calibration: signed 12-bit coefficient (stored as 16-bit)
    int16_t  dig_h5; // Humidity calibration: signed 12-bit coefficient (stored as 16-bit)
    int8_t   dig_h6; // Humidity calibration: signed 8-bit coefficient
    int32_t  t_fine; // Fine resolution temperature used in pressure/humidity compensation
};

/**
 * @struct bme280_uncomp_data
 * @brief Structure to hold uncompensated (raw) sensor data from the BME280.
 *
 * Description: Stores raw 20-bit pressure and temperature values, and 16-bit
 *              humidity value, as read directly from the sensor registers.
 */
struct bme280_uncomp_data
{
    uint32_t pressure;    // Raw pressure data (20-bit, stored in 32-bit)
    uint32_t temperature; // Raw temperature data (20-bit, stored in 32-bit)
    uint32_t humidity;    // Raw humidity data (16-bit, stored in 32-bit)
};

/*--------------------------------------------------------------------------*/
/* External Declarations                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @brief Global I2C instance for communication with the BME280 sensor.
 */
extern XIic IicInstance;

/**
 * @brief Global structure to store BME280 calibration data.
 */
extern struct bme280_calib_data calib_data;

/**
 * @brief Semaphore for synchronizing access to the BME280 sensor in FreeRTOS.
 */
extern SemaphoreHandle_t bme280_sem;

/*--------------------------------------------------------------------------*/
/* Function Prototypes                                                      */
/*--------------------------------------------------------------------------*/
/**
 * @brief Initialize the BME280 sensor and verify its presence.
 * @param iic Pointer to the XIic instance for I2C communication.
 * @return 0 on success, -1 on failure (e.g., wrong chip ID or I2C error).
 */
int bme_init ( XIic* iic );

/**
 * @brief Read calibration data from the BME280 sensor.
 * @param iic Pointer to the XIic instance for I2C communication.
 * @param calib Pointer to structure to store calibration data.
 * @return 0 on success.
 */
int bme_read_calibration_data ( XIic* iic, struct bme280_calib_data* calib );

/**
 * @brief FreeRTOS task to periodically read and process BME280 sensor data.
 * @param pvParameters Task parameters (unused in this implementation).
 */
void BME280_Task ( void* pvParameters );

/**
 * @brief Compensate raw temperature data using calibration coefficients.
 * @param uncomp_data Pointer to structure with raw sensor data.
 * @param calib_data Pointer to structure with calibration data.
 * @return Compensated temperature in hundredths of °C (e.g., 2530 = 25.30°C).
 */
int32_t compensate_temperature ( const struct bme280_uncomp_data* uncomp_data,
                                 struct bme280_calib_data*        calib_data );

/**
 * @brief Compensate raw pressure data using calibration coefficients.
 * @param uncomp_data Pointer to structure with raw sensor data.
 * @param calib_data Pointer to structure with calibration data.
 * @return Compensated pressure in Pascals (Pa).
 */
uint32_t compensate_pressure ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib_data );

/**
 * @brief Compensate raw humidity data using calibration coefficients.
 * @param uncomp_data Pointer to structure with raw sensor data.
 * @param calib_data Pointer to structure with calibration data.
 * @return Compensated humidity in 1024ths of % (e.g., 51200 = 50.00%).
 */
uint32_t compensate_humidity ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib_data );

#endif /* BME280_H */