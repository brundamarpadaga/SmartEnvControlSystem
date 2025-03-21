/*
 * ===========================================================================
 * File:    bme280.h
 * Summary: Header File for BME280 Sensor Driver in FreeRTOS System
 * ===========================================================================
 * Purpose: This header defines constants, structures, and function prototypes
 *          for interfacing with the BME280 environmental sensor (temperature,
 *          pressure, humidity) in a FreeRTOS-based embedded system. It includes
 *          calibration and data structures, as well as a custom sensor_Data type
 *          for integration with the broader system.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 * Author:  [Your Name/Group Name, if applicable]
 *
 * Dependencies:
 *   - stdint.h: Fixed-width integer types
 *   - FreeRTOS.h: FreeRTOS types and macros
 *   - semphr.h: SemaphoreHandle_t definition
 *   - xiic.h: Xilinx I2C driver types and functions
 *   - main.h: Provides external globals (IicInstance, calib_data, bme280_sem)
 *
 * Notes:
 *   - Designed for use with the BME280 sensor at I2C address 0x76.
 *   - Includes custom sensor_Data struct not part of the original BME280 spec.
 *   - All functions assume a FreeRTOS environment with I2C communication.
 */

/* ===========================================================================
 * Header Guard
 * ===========================================================================
 */
#ifndef BME280_H
#define BME280_H

/* ===========================================================================
 * Header Includes
 * ===========================================================================
 */
#include "FreeRTOS.h" // FreeRTOS types and macros (e.g., vTaskDelay)
#include "semphr.h"   // SemaphoreHandle_t for thread-safe I2C access
#include "xiic.h"     // Xilinx I2C driver for XIic type and functions
#include <stdint.h>   // Fixed-width integer types (e.g., uint16_t, int32_t)

/* ===========================================================================
 * General Macro Definitions
 * ===========================================================================
 */
#define BME280_I2C_ADDR 0x76      // I2C address of BME280 sensor (default)
#define REG_CHIP_ID 0xD0          // Register for chip ID (read-only)
#define REG_TEMP_PRESS_CALIB 0x88 // Start of temp/pressure calibration data
#define REG_HUMIDITY_CALIB 0xE1   // Start of humidity calibration data
#define REG_CTRL_HUM 0xF2         // Humidity oversampling control register
#define REG_CTRL_MEAS 0xF4        // Temp/pressure oversampling and mode control
#define REG_CONFIG 0xF5           // Standby time and filter coefficient config
#define REG_DATA 0xF7             // Start of measurement data (press, temp, hum)
#define BME280_CHIP_ID 0x60       // Expected chip ID value for BME280

/* ===========================================================================
 * Structure Definitions
 * ===========================================================================
 */

/**
 * @brief Calibration data structure for BME280 sensor.
 * @details Holds temperature, pressure, and humidity calibration coefficients
 *          read from the sensor’s non-volatile memory. Used in compensation
 *          functions to convert raw data to physical units.
 */
struct bme280_calib_data
{
    uint16_t dig_t1; // Temp calibration: unsigned coefficient 1
    int16_t  dig_t2; // Temp calibration: signed coefficient 2
    int16_t  dig_t3; // Temp calibration: signed coefficient 3
    uint16_t dig_p1; // Pressure calibration: unsigned coefficient 1
    int16_t  dig_p2; // Pressure calibration: signed coefficient 2
    int16_t  dig_p3; // Pressure calibration: signed coefficient 3
    int16_t  dig_p4; // Pressure calibration: signed coefficient 4
    int16_t  dig_p5; // Pressure calibration: signed coefficient 5
    int16_t  dig_p6; // Pressure calibration: signed coefficient 6
    int16_t  dig_p7; // Pressure calibration: signed coefficient 7
    int16_t  dig_p8; // Pressure calibration: signed coefficient 8
    int16_t  dig_p9; // Pressure calibration: signed coefficient 9
    uint8_t  dig_h1; // Humidity calibration: unsigned coefficient 1
    int16_t  dig_h2; // Humidity calibration: signed coefficient 2
    uint8_t  dig_h3; // Humidity calibration: unsigned coefficient 3
    int16_t  dig_h4; // Humidity calibration: signed coefficient 4
    int16_t  dig_h5; // Humidity calibration: signed coefficient 5
    int8_t   dig_h6; // Humidity calibration: signed coefficient 6
    int32_t  t_fine; // Fine temperature value used across compensations
};

/**
 * @brief Uncompensated data structure for BME280 sensor readings.
 * @details Stores raw 20-bit (pressure, temperature) and 16-bit (humidity)
 *          values from the sensor before compensation.
 */
struct bme280_uncomp_data
{
    uint32_t pressure;    // Raw pressure data (20 bits)
    uint32_t temperature; // Raw temperature data (20 bits)
    uint32_t humidity;    // Raw humidity data (16 bits)
};

/**
 * @brief Custom sensor data structure for system integration.
 * @details Extends BME280 data with luminosity for use in the main system,
 *          including compensated values in specific units.
 * @note Added for this project, not part of the standard BME280 library.
 */
typedef struct
{
    int32_t  temperature; // Compensated temperature in hundredths of °C (e.g., 2345 = 23.45°C)
    uint32_t pressure;    // Compensated pressure in Pascals (Pa)
    uint32_t humidity;    // Compensated humidity in 1024ths of % (e.g., 51200 = 50%)
    uint16_t luminosity;  // Luminosity in lux (from TSL2561, not BME280)
} sensor_Data;

/* ===========================================================================
 * External Declarations
 * ===========================================================================
 */
extern XIic                     IicInstance; // Global I2C instance from main.c
extern struct bme280_calib_data calib_data;  // Global calibration data from main.c
extern SemaphoreHandle_t        bme280_sem;  // Semaphore for BME280 access from main.c

/* ===========================================================================
 * Function Prototypes
 * ===========================================================================
 */

/**
 * @brief Initializes the BME280 sensor.
 * @param iic Pointer to the I2C instance.
 * @return int 0 on success, -1 on failure.
 */
int bme_init(XIic* iic);

/**
 * @brief Reads calibration data from the BME280 sensor.
 * @param iic Pointer to the I2C instance.
 * @param calib Pointer to the calibration data structure to fill.
 * @return int 0 on success (no error handling implemented).
 */
int bme_read_calibration_data(XIic* iic, struct bme280_calib_data* calib);

/**
 * @brief FreeRTOS task to periodically read BME280 sensor data.
 * @param pvParameters Pointer to the sensor_Data structure to update.
 */
void BME280_Task(void* pvParameters);

/**
 * @brief Compensates raw temperature data.
 * @param uncomp_data Pointer to uncompensated sensor data.
 * @param calib_data Pointer to calibration data.
 * @return int32_t Temperature in hundredths of °C.
 */
int32_t compensate_temperature(const struct bme280_uncomp_data* uncomp_data,
                               struct bme280_calib_data*        calib_data);

/**
 * @brief Compensates raw pressure data.
 * @param uncomp_data Pointer to uncompensated sensor data.
 * @param calib_data Pointer to calibration data.
 * @return uint32_t Pressure in Pascals (Pa).
 */
uint32_t compensate_pressure(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data);

/**
 * @brief Compensates raw humidity data.
 * @param uncomp_data Pointer to uncompensated sensor data.
 * @param calib_data Pointer to calibration data.
 * @return uint32_t Humidity in 1024ths of %.
 */
uint32_t compensate_humidity(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data);

#endif /* BME280_H */