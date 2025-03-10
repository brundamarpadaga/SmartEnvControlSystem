/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * [BSD-3-Clause license text omitted for brevity]
 *
 * @file    bme.h
 * @date    2025-03-10
 * @brief   Header file for BME280 sensor driver in a FreeRTOS-based system.
 */

#ifndef BME_H
#define BME_H

/*--------------------------------------------------------------------------*/
/* Header Includes                                                          */
/*--------------------------------------------------------------------------*/
#include <stdint.h> /* Fixed-width integer types */

#include "FreeRTOS.h" /* FreeRTOS types and macros */
#include "semphr.h"   /* SemaphoreHandle_t definition */
#include "xiic.h"     /* Xilinx I2C driver */

/*--------------------------------------------------------------------------*/
/* General Macro Definitions                                                */
/*--------------------------------------------------------------------------*/
#define BME280_I2C_ADDR 0x76
#define REG_CHIP_ID 0xD0
#define REG_TEMP_PRESS_CALIB 0x88
#define REG_HUMIDITY_CALIB 0xE1
#define REG_CTRL_HUM 0xF2
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_DATA 0xF7
#define BME280_CHIP_ID 0x60

/*--------------------------------------------------------------------------*/
/* Structure Definitions                                                    */
/*--------------------------------------------------------------------------*/
struct bme280_calib_data
{
    uint16_t dig_t1;
    int16_t  dig_t2;
    int16_t  dig_t3;
    uint16_t dig_p1;
    int16_t  dig_p2;
    int16_t  dig_p3;
    int16_t  dig_p4;
    int16_t  dig_p5;
    int16_t  dig_p6;
    int16_t  dig_p7;
    int16_t  dig_p8;
    int16_t  dig_p9;
    uint8_t  dig_h1;
    int16_t  dig_h2;
    uint8_t  dig_h3;
    int16_t  dig_h4;
    int16_t  dig_h5;
    int8_t   dig_h6;
    int32_t  t_fine;
};

struct bme280_uncomp_data
{
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
};

/*--------------------------------------------------------------------------*/
/* External Declarations                                                    */
/*--------------------------------------------------------------------------*/
extern XIic                     IicInstance;
extern struct bme280_calib_data calib_data;
extern SemaphoreHandle_t        bme280_sem;

/*--------------------------------------------------------------------------*/
/* Function Prototypes                                                      */
/*--------------------------------------------------------------------------*/
int      bme_init(XIic* iic);
int      bme_read_calibration_data(XIic* iic, struct bme280_calib_data* calib);
void     BME280_Task(void* pvParameters);
int32_t  compensate_temperature(const struct bme280_uncomp_data* uncomp_data,
                                struct bme280_calib_data*        calib_data);
uint32_t compensate_pressure(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data);
uint32_t compensate_humidity(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data);

#endif /* BME_H */
