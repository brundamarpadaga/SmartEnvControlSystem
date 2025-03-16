/*
 * bme280.c - BME280 sensor driver implementation for FreeRTOS
 *
 * Purpose: Provides functions to initialize, configure, and read data from the
 *          BME280 sensor, including compensation of raw data.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "bme280.h"

#include <stdlib.h>  // For abs()

#include "main.h"  // For i2c_sem, sensor_data, IicInstance, i2c_request_queue

int bme_init ( XIic* iic )
{
    uint8_t buffer[ 1 ];
    int     status;

    status = XIic_SetAddress ( iic, XII_ADDR_TO_SEND_TYPE, BME280_I2C_ADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Failed to set I2C address\r\n" );
        return -1;
    }

    buffer[ 0 ] = REG_CHIP_ID;
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_STOP );

    if ( buffer[ 0 ] != BME280_CHIP_ID )
    {
        xil_printf ( "[ERROR] Wrong Chip ID: 0x%02X\r\n", buffer[ 0 ] );
        return -1;
    }

    return 0;
}

int bme_read_calibration_data ( XIic* iic, struct bme280_calib_data* calib )
{
    uint8_t buffer[ 26 ];

    buffer[ 0 ] = REG_TEMP_PRESS_CALIB;
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 26, XIIC_STOP );

    calib->dig_t1 = (uint16_t) ( buffer[ 1 ] << 8 ) | buffer[ 0 ];
    calib->dig_t2 = (int16_t) ( buffer[ 3 ] << 8 ) | buffer[ 2 ];
    calib->dig_t3 = (int16_t) ( buffer[ 5 ] << 8 ) | buffer[ 4 ];
    calib->dig_p1 = (uint16_t) ( buffer[ 7 ] << 8 ) | buffer[ 6 ];
    calib->dig_p2 = (int16_t) ( buffer[ 9 ] << 8 ) | buffer[ 8 ];
    calib->dig_p3 = (int16_t) ( buffer[ 11 ] << 8 ) | buffer[ 10 ];
    calib->dig_p4 = (int16_t) ( buffer[ 13 ] << 8 ) | buffer[ 12 ];
    calib->dig_p5 = (int16_t) ( buffer[ 15 ] << 8 ) | buffer[ 14 ];
    calib->dig_p6 = (int16_t) ( buffer[ 17 ] << 8 ) | buffer[ 16 ];
    calib->dig_p7 = (int16_t) ( buffer[ 19 ] << 8 ) | buffer[ 18 ];
    calib->dig_p8 = (int16_t) ( buffer[ 21 ] << 8 ) | buffer[ 20 ];
    calib->dig_p9 = (int16_t) ( buffer[ 23 ] << 8 ) | buffer[ 22 ];
    calib->dig_h1 = buffer[ 25 ];

    buffer[ 0 ] = REG_HUMIDITY_CALIB;
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 7, XIIC_STOP );

    calib->dig_h2 = (int16_t) ( buffer[ 1 ] << 8 ) | buffer[ 0 ];
    calib->dig_h3 = buffer[ 2 ];
    calib->dig_h4 = (int16_t) ( (int8_t) buffer[ 3 ] * 16 ) | ( buffer[ 4 ] & 0x0F );
    calib->dig_h5 = (int16_t) ( (int8_t) buffer[ 5 ] * 16 ) | ( buffer[ 4 ] >> 4 );
    calib->dig_h6 = (int8_t) buffer[ 6 ];

    return 0;
}

void BME280_Task ( void* pvParameters )
{
    sensor_Data* sensor_data     = (sensor_Data*) pvParameters;
    xQueueHandle bme_reply_queue = xQueueCreate ( 1, sizeof ( float ) );
    uint8_t      buffer[ 26 ];

    if ( bme_reply_queue == NULL )
    {
        xil_printf ( "[ERROR] BME280 reply queue creation failed\r\n" );
        return;
    }

    // Initial configuration
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        buffer[ 0 ] = REG_CTRL_HUM;
        buffer[ 1 ] = 0x01;  // Humidity oversampling x1
        XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );
        buffer[ 0 ] = REG_CTRL_MEAS;
        buffer[ 1 ] = ( 0x5 << 5 ) | ( 0x1 << 2 ) | 0x3;  // Temp x4, Press x1, Normal mode
        XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );
        buffer[ 0 ] = REG_CONFIG;
        buffer[ 1 ] = ( 0x4 << 2 ) | ( 0x5 << 5 );  // Standby 500ms, Filter x16
        XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in BME280 init\r\n" );
    }

    vTaskDelay ( pdMS_TO_TICKS ( 500 ) );

    while ( 1 )
    {
        if ( i2c_request_queue == NULL || bme_reply_queue == NULL )
        {  // Cleanup check
            vQueueDelete ( bme_reply_queue );
            xil_printf ( "BME280 task exiting due to cleanup\r\n" );
            return;
        }

        i2c_request_t req = { .reply_queue = bme_reply_queue, .data = NULL, .len = 0, .cmd = 0 };
        float         temp, hum, press;

        req.type   = READ_BME280_TEMP;
        req.result = &temp;
        xQueueSend ( i2c_request_queue, &req, mainDONT_BLOCK );
        xQueueReceive ( bme_reply_queue, &temp, portMAX_DELAY );
        sensor_data->temperature = temp;

        req.type   = READ_BME280_HUM;
        req.result = &hum;
        xQueueSend ( i2c_request_queue, &req, mainDONT_BLOCK );
        xQueueReceive ( bme_reply_queue, &hum, portMAX_DELAY );
        sensor_data->humidity = hum;

        req.type   = READ_BME280_PRESS;
        req.result = &press;
        xQueueSend ( i2c_request_queue, &req, mainDONT_BLOCK );
        xQueueReceive ( bme_reply_queue, &press, portMAX_DELAY );
        sensor_data->pressure = press;

        vTaskDelay ( pdMS_TO_TICKS ( 1000 ) );  // Maintain 1s sampling rate
    }
}

/* Compensation Functions */
int32_t compensate_temperature ( const struct bme280_uncomp_data* uncomp_data,
                                 struct bme280_calib_data*        calib )
{
    int32_t var1, var2, T;
    int32_t temp_shifted = uncomp_data->temperature >> 3;
    int32_t t1_shifted   = (int32_t) calib->dig_t1 << 1;

    var1 = ( temp_shifted - t1_shifted ) * (int32_t) calib->dig_t2;
    var1 = var1 >> 11;

    temp_shifted      = uncomp_data->temperature >> 4;
    int32_t temp_diff = temp_shifted - (int32_t) calib->dig_t1;
    var2              = temp_diff * temp_diff;
    var2              = var2 >> 12;
    var2              = var2 * (int32_t) calib->dig_t3;
    var2              = var2 >> 14;

    int32_t t_fine = var1 + var2;
    T              = ( t_fine * 5 + 128 ) >> 8;  // Temperature in hundredths of C
    return T;
}

uint32_t compensate_pressure ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib )
{
    int32_t var1, var2;
    int64_t p;

    // Calculate t_fine
    int32_t temp_shifted = uncomp_data->temperature >> 3;
    int32_t t1_shifted   = (int32_t) calib->dig_t1 << 1;
    int32_t temp_diff    = temp_shifted - t1_shifted;
    int32_t term1        = temp_diff * (int32_t) calib->dig_t2;
    term1                = term1 >> 11;

    temp_shifted  = uncomp_data->temperature >> 4;
    temp_diff     = temp_shifted - (int32_t) calib->dig_t1;
    int32_t term2 = temp_diff * temp_diff;
    term2         = term2 >> 12;
    term2         = term2 * (int32_t) calib->dig_t3;
    term2         = term2 >> 14;

    int32_t t_fine = term1 + term2;

    // Pressure compensation
    var1 = ( (int64_t) t_fine >> 1 ) - (int64_t) 64000;
    var2 = var1 >> 2;
    var2 = ( var2 * var2 ) >> 11;
    var2 = var2 * (int32_t) calib->dig_p6;
    var2 = var2 + ( ( var1 * (int32_t) calib->dig_p5 ) << 1 );
    var2 = var2 >> 2;
    var2 = var2 + ( (int32_t) calib->dig_p4 << 16 );

    int32_t var1_temp    = var1 >> 2;
    int32_t var1_squared = var1_temp * var1_temp;
    var1_squared         = var1_squared >> 13;
    int32_t term3        = calib->dig_p3 * var1_squared;
    term3                = term3 >> 3;
    int32_t term4        = (int32_t) calib->dig_p2 * var1;
    term4                = term4 >> 1;
    var1                 = ( term3 + term4 ) >> 18;
    var1                 = ( ( 32768 + var1 ) * (int32_t) calib->dig_p1 ) >> 15;

    if ( var1 == 0 )
    {
        return 0;  // Avoid division by zero
    }

    p = (int64_t) 1048576 - uncomp_data->pressure;
    p = p - ( var2 >> 12 );
    p = p * 3125;
    if ( p < 0x80000000 )
    {
        p = ( p << 1 ) / (uint32_t) var1;
    }
    else
    {
        p = ( p / (uint32_t) var1 ) * 2;
    }

    int32_t p_shifted = (int32_t) ( p >> 3 );
    int32_t p_squared = p_shifted * p_shifted;
    p_squared         = p_squared >> 13;
    var1              = (int32_t) calib->dig_p9 * p_squared;
    var1              = var1 >> 12;

    var2 = (int32_t) ( p >> 2 ) * (int32_t) calib->dig_p8;
    var2 = var2 >> 13;

    p = p + ( ( var1 + var2 + calib->dig_p7 ) >> 4 );
    return (uint32_t) p;  // Pressure in Pa
}

uint32_t compensate_humidity ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib )
{
    int32_t v_x1_u32r;

    // Calculate t_fine
    int32_t temp_shifted = uncomp_data->temperature >> 3;
    int32_t t1_shifted   = (int32_t) calib->dig_t1 << 1;
    int32_t temp_diff    = temp_shifted - t1_shifted;
    int32_t term1        = temp_diff * (int32_t) calib->dig_t2;
    term1                = term1 >> 11;

    temp_shifted  = uncomp_data->temperature >> 4;
    temp_diff     = temp_shifted - (int32_t) calib->dig_t1;
    int32_t term2 = temp_diff * temp_diff;
    term2         = term2 >> 12;
    term2         = term2 * (int32_t) calib->dig_t3;
    term2         = term2 >> 14;

    int32_t t_fine = term1 + term2;

    // Humidity compensation
    v_x1_u32r = t_fine - (int32_t) 76800;

    int32_t hum_shifted = uncomp_data->humidity << 14;
    int32_t h4_shifted  = (int32_t) calib->dig_h4 << 20;
    int32_t h5_term     = (int32_t) calib->dig_h5 * v_x1_u32r;
    int32_t term3       = hum_shifted - h4_shifted - h5_term;
    term3               = term3 + (int32_t) 16384;
    term3               = term3 >> 15;

    int32_t h6_term = v_x1_u32r * (int32_t) calib->dig_h6;
    h6_term         = h6_term >> 10;
    int32_t h3_term = v_x1_u32r * (int32_t) calib->dig_h3;
    h3_term         = h3_term >> 11;
    h3_term         = h3_term + (int32_t) 32768;
    int32_t term4   = h6_term * h3_term;
    term4           = term4 >> 10;
    term4           = term4 + (int32_t) 2097152;
    term4           = term4 * (int32_t) calib->dig_h2;
    term4           = term4 + 8192;
    term4           = term4 >> 14;

    v_x1_u32r = term3 * term4;

    int32_t v_squared = v_x1_u32r >> 15;
    v_squared         = v_squared * v_squared;
    v_squared         = v_squared >> 7;
    int32_t h1_term   = v_squared * (int32_t) calib->dig_h1;
    h1_term           = h1_term >> 4;
    v_x1_u32r         = v_x1_u32r - h1_term;

    v_x1_u32r = ( v_x1_u32r < 0 ) ? 0 : v_x1_u32r;
    v_x1_u32r = ( v_x1_u32r > 419430400 ) ? 419430400 : v_x1_u32r;
    return (uint32_t) ( v_x1_u32r >> 12 );  // Humidity in 1024ths of %
}