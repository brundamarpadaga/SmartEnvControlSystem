/*
 * bme280.c - BME280 sensor driver implementation for FreeRTOS
 *
 * Purpose: Provides functions to initialize, configure, and read data from the
 *          BME280 sensor, including compensation of raw data, optimized for minimal memory use.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "bme280.h"

/*
 * bme_init - Initialize the BME280 sensor over I2C
 */
int bme_init(XIic* iic)
{
    uint8_t buffer;
    int     status;

    status = XIic_SetAddress(iic, XII_ADDR_TO_SEND_TYPE, BME280_I2C_ADDR);
    if (status != XST_SUCCESS)
    {
        return -1;
    }

    buffer = REG_CHIP_ID;
    XIic_Send(iic->BaseAddress, BME280_I2C_ADDR, &buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(iic->BaseAddress, BME280_I2C_ADDR, &buffer, 1, XIIC_STOP);

    if (buffer != BME280_CHIP_ID)
    {
        return -1;
    }

    return 0;
}

/*
 * bme_read_calibration_data - Retrieve calibration data from BME280
 */
int bme_read_calibration_data(XIic* iic, struct bme280_calib_data* calib)
{
    uint8_t buffer[26]; /* Reduced from 26 to reuse for both reads */

    buffer[0] = REG_TEMP_PRESS_CALIB;
    XIic_Send(iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(iic->BaseAddress, BME280_I2C_ADDR, buffer, 26, XIIC_STOP);

    calib->dig_t1 = (uint16_t) (buffer[1] << 8) | buffer[0];
    calib->dig_t2 = (int16_t) (buffer[3] << 8) | buffer[2];
    calib->dig_t3 = (int16_t) (buffer[5] << 8) | buffer[4];
    calib->dig_p1 = (uint16_t) (buffer[7] << 8) | buffer[6];
    calib->dig_p2 = (int16_t) (buffer[9] << 8) | buffer[8];
    calib->dig_p3 = (int16_t) (buffer[11] << 8) | buffer[10];
    calib->dig_p4 = (int16_t) (buffer[13] << 8) | buffer[12];
    calib->dig_p5 = (int16_t) (buffer[15] << 8) | buffer[14];
    calib->dig_p6 = (int16_t) (buffer[17] << 8) | buffer[16];
    calib->dig_p7 = (int16_t) (buffer[19] << 8) | buffer[18];
    calib->dig_p8 = (int16_t) (buffer[21] << 8) | buffer[20];
    calib->dig_p9 = (int16_t) (buffer[23] << 8) | buffer[22];
    calib->dig_h1 = buffer[25];

    buffer[0] = REG_HUMIDITY_CALIB;
    XIic_Send(iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(iic->BaseAddress, BME280_I2C_ADDR, buffer, 7, XIIC_STOP);

    calib->dig_h2 = (int16_t) (buffer[1] << 8) | buffer[0];
    calib->dig_h3 = buffer[2];
    calib->dig_h4 = (int16_t) ((int8_t) buffer[3] * 16) | (buffer[4] & 0x0F);
    calib->dig_h5 = (int16_t) ((int8_t) buffer[5] * 16) | (buffer[4] >> 4);
    calib->dig_h6 = (int8_t) buffer[6];

    return 0;
}

/*
 * compensate_temperature - Compensate raw temperature data with fewer variables
 */
int32_t compensate_temperature(const struct bme280_uncomp_data* uncomp_data,
                               struct bme280_calib_data*        calib_data)
{
    int32_t var1 = (int32_t) ((uncomp_data->temperature / 8) - ((int32_t) calib_data->dig_t1 * 2));
    var1         = (var1 * ((int32_t) calib_data->dig_t2)) / 2048;

    int32_t var2 = (int32_t) ((uncomp_data->temperature / 16) - ((int32_t) calib_data->dig_t1));
    var2         = (((var2 * var2) / 4096) * ((int32_t) calib_data->dig_t3)) / 16384;

    calib_data->t_fine  = var1 + var2;
    int32_t temperature = (calib_data->t_fine * 5 + 128) / 256;

    if (temperature < -4000)
        return -4000;
    if (temperature > 8500)
        return 8500;
    return temperature;
}

/*
 * compensate_pressure - Compensate raw pressure data with fewer variables
 */
uint32_t compensate_pressure(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data)
{
    int32_t var1 = (((int32_t) calib_data->t_fine) / 2) - 64000;
    int32_t var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) calib_data->dig_p6);
    var2         = var2 + ((var1 * ((int32_t) calib_data->dig_p5)) * 2);
    var2         = (var2 / 4) + (((int32_t) calib_data->dig_p4) * 65536);

    var1 = (((calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8) +
            (((int32_t) calib_data->dig_p2) * var1) / 2) /
           262144;
    var1 = (((32768 + var1)) * ((int32_t) calib_data->dig_p1)) / 32768;

    if (var1)
    {
        uint32_t pressure =
            ((uint32_t) (1048576 - uncomp_data->pressure) - (uint32_t) (var2 / 4096)) * 3125;
        if (pressure < 0x80000000)
            pressure = (pressure << 1) / ((uint32_t) var1);
        else
            pressure = (pressure / (uint32_t) var1) * 2;

        var1 = (((int32_t) calib_data->dig_p9) *
                ((int32_t) (((pressure / 8) * (pressure / 8)) / 8192))) /
               4096;
        var2     = (((int32_t) (pressure / 4)) * ((int32_t) calib_data->dig_p8)) / 8192;
        pressure = (uint32_t) ((int32_t) pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

        if (pressure < 30000)
            return 30000;
        if (pressure > 110000)
            return 110000;
        return pressure;
    }
    return 30000;
}

/*
 * compensate_humidity - Compensate raw humidity data with fewer variables
 */
uint32_t compensate_humidity(const struct bme280_uncomp_data* uncomp_data,
                             const struct bme280_calib_data*  calib_data)
{
    int32_t var1 = calib_data->t_fine - 76800;
    int32_t var2 = (uncomp_data->humidity * 16384 - ((int32_t) calib_data->dig_h4 * 1048576) -
                    ((int32_t) calib_data->dig_h5 * var1) + 16384) /
                   32768;

    int32_t temp = (var1 * ((int32_t) calib_data->dig_h6)) / 1024;
    int32_t var3 = (var1 * ((int32_t) calib_data->dig_h3)) / 2048;
    var3         = ((temp * (var3 + 32768)) / 1024) + 2097152;

    var2 = (var2 * var3 * ((int32_t) calib_data->dig_h2) + 8192) / 16384;
    temp = (var2 / 32768) * (var2 / 32768);
    var2 -= (temp * ((int32_t) calib_data->dig_h1)) / 16;

    if (var2 < 0)
        var2 = 0;
    if (var2 > 419430400)
        var2 = 419430400;

    uint32_t humidity = (uint32_t) (var2 / 4096);
    if (humidity > 102400)
        return 102400;
    return humidity;
}
