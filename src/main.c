/*
 * main.c - FreeRTOS-based BME280 test
 *
 * Purpose: Initializes the BME280 sensor via I2C within a FreeRTOS task,
 *          reads temperature, pressure, and humidity, and prints them over UART.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors:
 */

#include "FreeRTOS.h"
#include "bme280_defs.h" /* Only need definitions for structs and constants */
#include "bme280.h"
#include "i2c.h"         /* Your I2C driver header */
#include "semphr.h"
#include "sleep.h"
#include "task.h"
#include "xiic.h"
#include "xil_printf.h"
#include <stdlib.h>



/* Global I2C instance */
XIic IicInstance;
/* Calibration data */
struct bme280_calib_data calib_data;
/* Semaphore for sensor read synchronization */
SemaphoreHandle_t bme280_sem;



/* BME280 sensor task */
void bme280_task(void* pvParameters)
{
    struct bme280_uncomp_data uncomp_data;


    bme280_configure(&IicInstance);
    /* Wait for initial measurement */
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        /* Wait for semaphore to ensure exclusive access */
        if (xSemaphoreTake(bme280_sem, pdMS_TO_TICKS(100)) == pdTRUE)
        {

        	bme_read8Bytes(&IicInstance, &uncomp_data, &calib_data);

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

    xil_printf("BME280 FreeRTOS Test Starting\r\n");

    bme280_initialise(&IicInstance);
    calib_data = readTempPressureHumidityCalib(&IicInstance);


    /* Create semaphore */
    bme280_sem = xSemaphoreCreateBinary();
    if (bme280_sem == NULL)
    {
        xil_printf("[ERROR] Semaphore creation failed\r\n");
        return -1;
    }
    xSemaphoreGive(bme280_sem); /* Initially available */

    /* Create BME280 task */
    xTaskCreate( bme280_task, "BME280", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    /* Start the scheduler */
    xil_printf("Starting FreeRTOS scheduler\r\n");
    vTaskStartScheduler();

    /* Should never reach here */
    xil_printf("[ERROR] Scheduler failed\r\n");
    return -1;
}
