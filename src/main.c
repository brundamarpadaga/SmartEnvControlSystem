/*
 * main.c - FreeRTOS-based BME280 test
 *
 * Purpose: Initializes the BME280 sensor via I2C within a FreeRTOS task,
 *          reads temperature, pressure, and humidity, and prints them over UART.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "FreeRTOS.h"
#include "bme280_defs.h" /* Only need definitions for structs and constants */
#include "i2c.h"         /* Your I2C driver header */
#include "semphr.h"
#include "sleep.h"
#include "task.h"
#include "xiic.h"
#include "xil_printf.h"

/* BME280 I2C address */
#define BME280_I2C_ADDR 0x76

/* Register addresses */
#define REG_CHIP_ID 0xD0
#define REG_TEMP_PRESS_CALIB 0x88
#define REG_HUMIDITY_CALIB 0xE1
#define REG_CTRL_HUM 0xF2
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_DATA 0xF7

/* Global I2C instance */
XIic IicInstance;
/* Calibration data */
struct bme280_calib_data calib_data;
/* Semaphore for sensor read synchronization */
SemaphoreHandle_t bme280_sem;

/* Compensation functions (extracted from bme280.c) */
static int32_t compensate_temperature(const struct bme280_uncomp_data* uncomp_data,
                                      struct bme280_calib_data*        calib_data)
{
    int32_t var1, var2, temperature;
    var1 = (int32_t) ((uncomp_data->temperature / 8) - ((int32_t) calib_data->dig_t1 * 2));
    var1 = (var1 * ((int32_t) calib_data->dig_t2)) / 2048;
    var2 = (int32_t) ((uncomp_data->temperature / 16) - ((int32_t) calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t) calib_data->dig_t3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature        = (calib_data->t_fine * 5 + 128) / 256;
    if (temperature < -4000)
        temperature = -4000;
    else if (temperature > 8500)
        temperature = 8500;
    return temperature;
}

static uint32_t compensate_pressure(const struct bme280_uncomp_data* uncomp_data,
                                    const struct bme280_calib_data*  calib_data)
{
    int32_t  var1, var2, var3, var4;
    uint32_t var5, pressure;
    var1 = (((int32_t) calib_data->t_fine) / 2) - (int32_t) 64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) calib_data->dig_p6);
    var2 = var2 + ((var1 * ((int32_t) calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t) calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t) calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t) calib_data->dig_p1)) / 32768;
    if (var1)
    {
        var5     = (uint32_t) ((uint32_t) 1048576) - uncomp_data->pressure;
        pressure = ((uint32_t) (var5 - (uint32_t) (var2 / 4096))) * 3125;
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
            pressure = 30000;
        else if (pressure > 110000)
            pressure = 110000;
    }
    else
    {
        pressure = 30000;
    }
    return pressure;
}

static uint32_t compensate_humidity(const struct bme280_uncomp_data* uncomp_data,
                                    const struct bme280_calib_data*  calib_data)
{
    int32_t  var1, var2, var3, var4, var5;
    uint32_t humidity;
    var1     = calib_data->t_fine - ((int32_t) 76800);
    var2     = (int32_t) (uncomp_data->humidity * 16384);
    var3     = (int32_t) (((int32_t) calib_data->dig_h4) * 1048576);
    var4     = ((int32_t) calib_data->dig_h5) * var1;
    var5     = (((var2 - var3) - var4) + (int32_t) 16384) / 32768;
    var2     = (var1 * ((int32_t) calib_data->dig_h6)) / 1024;
    var3     = (var1 * ((int32_t) calib_data->dig_h3)) / 2048;
    var4     = ((var2 * (var3 + (int32_t) 32768)) / 1024) + (int32_t) 2097152;
    var2     = ((var4 * ((int32_t) calib_data->dig_h2)) + 8192) / 16384;
    var3     = var5 * var2;
    var4     = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5     = var3 - ((var4 * ((int32_t) calib_data->dig_h1)) / 16);
    var5     = (var5 < 0 ? 0 : var5);
    var5     = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t) (var5 / 4096);
    if (humidity > 102400)
        humidity = 102400;
    return humidity;
}

/* BME280 sensor task */
void bme280_task(void* pvParameters)
{
    struct bme280_uncomp_data uncomp_data;
    uint8_t                   buffer[26];
    int                       status;

    /* Configure sensor */
    buffer[0] = REG_CTRL_HUM;
    buffer[1] = 0x01; /* osr_h = 1x */
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);

    buffer[0] = REG_CTRL_MEAS;
    buffer[1] = (0x5 << 5) | (0x1 << 2) | 0x3; /* osr_t = 2x, osr_p = 16x, mode = normal */
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);

    buffer[0] = REG_CONFIG;
    buffer[1] = (0x4 << 2) | (0x5 << 5); /* filter = 16, t_sb = 1000ms */
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);

    /* Wait for initial measurement */
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        /* Wait for semaphore to ensure exclusive access */
        if (xSemaphoreTake(bme280_sem, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            /* Read 8 bytes of data */
            buffer[0] = REG_DATA;
            status =
                XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
            status += XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP);

            if (status != 9)
            {
                xil_printf("[ERROR] Data read failed, status: %d\r\n", status);
            }
            else
            {
                /* Parse raw data */
                uncomp_data.pressure =
                    ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
                uncomp_data.temperature =
                    ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
                uncomp_data.humidity = ((uint32_t) buffer[6] << 8) | buffer[7];

                /* Debug raw data */
                xil_printf("Raw T: %u, P: %u, H: %u\r\n",
                           uncomp_data.temperature,
                           uncomp_data.pressure,
                           uncomp_data.humidity);

                /* Compensate data */
                int32_t  temp  = compensate_temperature(&uncomp_data, &calib_data);
                uint32_t press = compensate_pressure(&uncomp_data, &calib_data);
                uint32_t hum   = compensate_humidity(&uncomp_data, &calib_data);

                /* Manual formatting for xil_printf (no %f support) */
                xil_printf("Temp: %d.%02d C, Press: %u.%02u hPa, Hum: %u.%02u%%\r\n",
                           temp / 100,
                           abs(temp % 100), /* Temp in °C */
                           press / 100,
                           press % 100, /* Pressure in hPa */
                           hum / 1024,
                           (hum % 1024) * 100 / 1024); /* Humidity in % */
            }

            /* Release semaphore */
            xSemaphoreGive(bme280_sem);
        }
        else
        {
            xil_printf("[ERROR] Semaphore timeout\r\n");
        }

        /* Delay 1 second */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    int     status;
    uint8_t buffer[26];

    xil_printf("BME280 FreeRTOS Test Starting\r\n");

    /* Initialize I2C */
    status = i2c_init();
    if (status != XST_SUCCESS)
    {
        xil_printf("[ERROR] I2C Initialization failed\r\n");
        return -1;
    }

    /* Set BME280 I2C address */
    if (XIic_SetAddress(&IicInstance, XII_ADDR_TO_SEND_TYPE, BME280_I2C_ADDR) != XST_SUCCESS)
    {
        xil_printf("[ERROR] Failed to set I2C address\r\n");
        return -1;
    }

    /* Verify chip ID */
    buffer[0] = REG_CHIP_ID;
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_STOP);
    if (buffer[0] != 0x60)
    {
        xil_printf("[ERROR] Wrong Chip ID: 0x%02X\r\n", buffer[0]);
        return -1;
    }

    /* Read temperature and pressure calibration data (26 bytes) */
    buffer[0] = REG_TEMP_PRESS_CALIB;
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 26, XIIC_STOP);
    calib_data.dig_t1 = (uint16_t) (buffer[1] << 8) | buffer[0];
    calib_data.dig_t2 = (int16_t) (buffer[3] << 8) | buffer[2];
    calib_data.dig_t3 = (int16_t) (buffer[5] << 8) | buffer[4];
    calib_data.dig_p1 = (uint16_t) (buffer[7] << 8) | buffer[6];
    calib_data.dig_p2 = (int16_t) (buffer[9] << 8) | buffer[8];
    calib_data.dig_p3 = (int16_t) (buffer[11] << 8) | buffer[10];
    calib_data.dig_p4 = (int16_t) (buffer[13] << 8) | buffer[12];
    calib_data.dig_p5 = (int16_t) (buffer[15] << 8) | buffer[14];
    calib_data.dig_p6 = (int16_t) (buffer[17] << 8) | buffer[16];
    calib_data.dig_p7 = (int16_t) (buffer[19] << 8) | buffer[18];
    calib_data.dig_p8 = (int16_t) (buffer[21] << 8) | buffer[20];
    calib_data.dig_p9 = (int16_t) (buffer[23] << 8) | buffer[22];
    calib_data.dig_h1 = buffer[25];

    /* Read humidity calibration data (7 bytes) */
    buffer[0] = REG_HUMIDITY_CALIB;
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
    XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 7, XIIC_STOP);
    calib_data.dig_h2 = (int16_t) (buffer[1] << 8) | buffer[0];
    calib_data.dig_h3 = buffer[2];
    calib_data.dig_h4 = (int16_t) ((int8_t) buffer[3] * 16) | (buffer[4] & 0x0F);
    calib_data.dig_h5 = (int16_t) ((int8_t) buffer[5] * 16) | (buffer[4] >> 4);
    calib_data.dig_h6 = (int8_t) buffer[6];

    /* Create semaphore */
    bme280_sem = xSemaphoreCreateBinary();
    if (bme280_sem == NULL)
    {
        xil_printf("[ERROR] Semaphore creation failed\r\n");
        return -1;
    }
    xSemaphoreGive(bme280_sem); /* Initially available */

    /* Create BME280 task */
    xTaskCreate(
        bme280_task, "BME280", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    /* Start the scheduler */
    xil_printf("Starting FreeRTOS scheduler\r\n");
    vTaskStartScheduler();

    /* Should never reach here */
    xil_printf("[ERROR] Scheduler failed\r\n");
    return -1;
}
