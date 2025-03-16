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

#include "main.h"  // For i2c_sem, sensor_data, IicInstance

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
	sensor_Data* sensor_data = (sensor_Data*) pvParameters;  // Cast parameter to access sensor data
    struct bme280_uncomp_data uncomp_data;
    uint8_t                   buffer[ 26 ];
    int                       status;

    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        buffer[ 0 ] = REG_CTRL_HUM;
        buffer[ 1 ] = 0x01;
        XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );
        buffer[ 0 ] = REG_CTRL_MEAS;
        buffer[ 1 ] = ( 0x5 << 5 ) | ( 0x1 << 2 ) | 0x3;
        XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );
        buffer[ 0 ] = REG_CONFIG;
        buffer[ 1 ] = ( 0x4 << 2 ) | ( 0x5 << 5 );
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
        if ( xSemaphoreTake ( bme280_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
        {
            if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
            {
                buffer[ 0 ] = REG_DATA;
                status      = XIic_Send (
                    IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
                status +=
                    XIic_Recv ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP );

                if ( status != 9 )
                {
                    xil_printf ( "[ERROR] BME280 data read failed, status: %d\r\n", status );
                }
                else
                {
                    uncomp_data.pressure = ( (uint32_t) buffer[ 0 ] << 12 ) |
                                           ( (uint32_t) buffer[ 1 ] << 4 ) | ( buffer[ 2 ] >> 4 );
                    uncomp_data.temperature = ( (uint32_t) buffer[ 3 ] << 12 ) |
                                              ( (uint32_t) buffer[ 4 ] << 4 ) |
                                              ( buffer[ 5 ] >> 4 );
                    uncomp_data.humidity = ( (uint32_t) buffer[ 6 ] << 8 ) | buffer[ 7 ];

                    sensor_data->temperature = compensate_temperature ( &uncomp_data, &calib_data );
                    sensor_data->pressure    = compensate_pressure ( &uncomp_data, &calib_data );
                    sensor_data->humidity    = compensate_humidity ( &uncomp_data, &calib_data );
                }
                xSemaphoreGive ( i2c_sem );
            }
            else
            {
                xil_printf ( "[ERROR] I2C semaphore timeout in BME280_Task\r\n" );
            }
            xSemaphoreGive ( bme280_sem );
        }
        vTaskDelay ( pdMS_TO_TICKS ( 1000 ) );
    }
}

/* Compensation Functions */
int32_t compensate_temperature ( const struct bme280_uncomp_data* uncomp_data,
                                 struct bme280_calib_data*        calib )
{
    int32_t var1, var2, T;

    var1 = ( ( ( ( uncomp_data->temperature >> 3 ) - ( (int32_t) calib->dig_t1 << 1 ) ) ) *
             ( (int32_t) calib->dig_t2 ) ) >>
           11;
    var2 = ( ( ( ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) *
                 ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) ) >>
               12 ) *
             ( (int32_t) calib->dig_t3 ) ) >>
           14;
    int32_t t_fine = var1 + var2;
    T              = ( t_fine * 5 + 128 ) >> 8;  // Temperature in hundredths of C
    return T;
}

uint32_t compensate_pressure ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib )
{
    int32_t var1, var2;
    int64_t p;

    int32_t t_fine =
        ( ( ( ( ( uncomp_data->temperature >> 3 ) - ( (int32_t) calib->dig_t1 << 1 ) ) ) *
            ( (int32_t) calib->dig_t2 ) ) >>
          11 ) +
        ( ( ( ( ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) *
                ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) ) >>
              12 ) *
            ( (int32_t) calib->dig_t3 ) ) >>
          14 );

    var1 = ( ( (int64_t) t_fine ) >> 1 ) - (int64_t) 64000;
    var2 = ( ( ( var1 >> 2 ) * ( var1 >> 2 ) ) >> 11 ) * ( (int32_t) calib->dig_p6 );
    var2 = var2 + ( ( var1 * ( (int32_t) calib->dig_p5 ) ) << 1 );
    var2 = ( var2 >> 2 ) + ( ( (int32_t) calib->dig_p4 ) << 16 );
    var1 = ( ( ( calib->dig_p3 * ( ( ( var1 >> 2 ) * ( var1 >> 2 ) ) >> 13 ) ) >> 3 ) +
             ( ( ( (int32_t) calib->dig_p2 ) * var1 ) >> 1 ) ) >>
           18;
    var1 = ( ( ( ( 32768 + var1 ) ) * ( (int32_t) calib->dig_p1 ) ) >> 15 );

    if ( var1 == 0 )
    {
        return 0;  // Avoid division by zero
    }

    p = ( ( (uint32_t) ( ( (int64_t) 1048576 ) - uncomp_data->pressure ) - ( var2 >> 12 ) ) ) *
        3125;
    if ( p < 0x80000000 )
    {
        p = ( p << 1 ) / ( (uint32_t) var1 );
    }
    else
    {
        p = ( p / (uint32_t) var1 ) * 2;
    }

    var1 =
        ( ( (int32_t) calib->dig_p9 ) * ( (int32_t) ( ( ( p >> 3 ) * ( p >> 3 ) ) >> 13 ) ) ) >> 12;
    var2 = ( ( (int32_t) ( p >> 2 ) ) * ( (int32_t) calib->dig_p8 ) ) >> 13;
    p    = (uint32_t) ( (int64_t) p + ( ( var1 + var2 + calib->dig_p7 ) >> 4 ) );
    return p;  // Pressure in Pa
}

uint32_t compensate_humidity ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib )
{
    int32_t v_x1_u32r;
    int32_t t_fine =
        ( ( ( ( ( uncomp_data->temperature >> 3 ) - ( (int32_t) calib->dig_t1 << 1 ) ) ) *
            ( (int32_t) calib->dig_t2 ) ) >>
          11 ) +
        ( ( ( ( ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) *
                ( ( uncomp_data->temperature >> 4 ) - ( (int32_t) calib->dig_t1 ) ) ) >>
              12 ) *
            ( (int32_t) calib->dig_t3 ) ) >>
          14 );

    v_x1_u32r = ( t_fine - ( (int32_t) 76800 ) );
    v_x1_u32r = ( ( ( ( ( uncomp_data->humidity << 14 ) - ( ( (int32_t) calib->dig_h4 ) << 20 ) -
                        ( ( (int32_t) calib->dig_h5 ) * v_x1_u32r ) ) +
                      ( (int32_t) 16384 ) ) >>
                    15 ) *
                  ( ( ( ( ( ( ( v_x1_u32r * ( (int32_t) calib->dig_h6 ) ) >> 10 ) *
                            ( ( ( v_x1_u32r * ( (int32_t) calib->dig_h3 ) ) >> 11 ) +
                              ( (int32_t) 32768 ) ) ) >>
                          10 ) +
                        ( (int32_t) 2097152 ) ) *
                          ( (int32_t) calib->dig_h2 ) +
                      8192 ) >>
                    14 ) );
    v_x1_u32r = ( v_x1_u32r - ( ( ( ( ( v_x1_u32r >> 15 ) * ( v_x1_u32r >> 15 ) ) >> 7 ) *
                                  ( (int32_t) calib->dig_h1 ) ) >>
                                4 ) );
    v_x1_u32r = ( v_x1_u32r < 0 ) ? 0 : v_x1_u32r;
    v_x1_u32r = ( v_x1_u32r > 419430400 ) ? 419430400 : v_x1_u32r;
    return (uint32_t) ( v_x1_u32r >> 12 );  // Humidity in 1024ths of %
}
