/*
 * lcd.c - Driver implementation for ELEGOO 0.96-inch OLED (SSD1306)
 *
 * Purpose: Provides functions to initialize and display data on the OLED
 *          over I2C, integrated with FreeRTOS for task-based updates.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "lcd.h"
#include "main.h"  // For i2c_sem, IicInstance, sensor_data
#include <stdio.h>

extern const uint8_t ssd1306xled_font8x16[];

/* Write a command to the OLED */
static void oled_write_cmd(XIic* i2c, uint8_t cmd) {
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t buffer[2] = {OLED_CMD, cmd};
        int status = XIic_Send(i2c->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
        if (status != 2) {
            xil_printf("[ERROR] OLED cmd write failed: %d\r\n", status);
        }
        xSemaphoreGive(i2c_sem);
    } else {
        xil_printf("[ERROR] I2C semaphore timeout in oled_write_cmd\r\n");
    }
}

/* Write data to the OLED, limited to 32 bytes per transaction */
static void oled_write_data(XIic* i2c, uint8_t* data, uint32_t len) {
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t buffer[33];  // 32 bytes data + 1 control byte
        buffer[0] = OLED_DATA;
        uint32_t bytes_to_send = (len > 32) ? 32 : len;  // Cap at 32 bytes
        for (uint32_t i = 0; i < bytes_to_send; i++) {
            buffer[i + 1] = data[i];
        }
        int status = XIic_Send(i2c->BaseAddress, OLED_I2C_ADDR, buffer, bytes_to_send + 1, XIIC_STOP);
        if (status != bytes_to_send + 1) {
            xil_printf("[ERROR] OLED data write failed: %d\r\n", status);
        }
        xSemaphoreGive(i2c_sem);
    } else {
        xil_printf("[ERROR] I2C semaphore timeout in oled_write_data\r\n");
    }
}

int lcd_init(XIic* i2c) {
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        int status = XIic_SetAddress(i2c, XII_ADDR_TO_SEND_TYPE, OLED_I2C_ADDR);
        if (status != XST_SUCCESS) {
            xil_printf("[ERROR] Failed to set OLED I2C address\r\n");
            xSemaphoreGive(i2c_sem);
            return XST_FAILURE;
        }
        xSemaphoreGive(i2c_sem);
    } else {
        xil_printf("[ERROR] I2C semaphore timeout in lcd_init\r\n");
        return XST_FAILURE;
    }

    oled_write_cmd(i2c, 0xAE);  // Display OFF
    oled_write_cmd(i2c, 0xD5); oled_write_cmd(i2c, 0x80);
    oled_write_cmd(i2c, 0xA8); oled_write_cmd(i2c, 0x3F);
    oled_write_cmd(i2c, 0xD3); oled_write_cmd(i2c, 0x00);
    oled_write_cmd(i2c, 0x40);
    oled_write_cmd(i2c, 0x8D); oled_write_cmd(i2c, 0x14);
    oled_write_cmd(i2c, 0x20); oled_write_cmd(i2c, 0x00);
    oled_write_cmd(i2c, 0xA0);
    oled_write_cmd(i2c, 0xC0);
    oled_write_cmd(i2c, 0xDA); oled_write_cmd(i2c, 0x12);
    oled_write_cmd(i2c, 0x81); oled_write_cmd(i2c, 0xCF);
    oled_write_cmd(i2c, 0xD9); oled_write_cmd(i2c, 0xF1);
    oled_write_cmd(i2c, 0xDB); oled_write_cmd(i2c, 0x40);
    oled_write_cmd(i2c, 0xA4);
    oled_write_cmd(i2c, 0xA6);

    /* Clear the entire display */
    uint8_t clear_buffer[32] = {0};
    for (uint8_t page = 0; page < 8; page++) {
        oled_write_cmd(i2c, 0xB0 + page);
        oled_write_cmd(i2c, 0x00);
        oled_write_cmd(i2c, 0x10);
        for (uint8_t i = 0; i < 128; i += 32) {
            oled_write_data(i2c, clear_buffer, 32);
        }
    }

    oled_write_cmd(i2c, 0xAF);  // Display ON
    return XST_SUCCESS;
}

/* Display a string on the OLED at a specified page */
int lcd_display_string(XIic* i2c, const char* str, uint8_t page) {
    if (page > 7) return XST_FAILURE;

    if (xSemaphoreTake(oled_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        /* Clear the two pages (16 pixels tall per character) */
        uint8_t clear_buffer[32] = {0};
        for (uint8_t i = 0; i < 128; i += 32) {
            oled_write_cmd(i2c, 0xB0 + page);     // Set page
            oled_write_cmd(i2c, 0x00 + (i & 0x0F));  // Column low
            oled_write_cmd(i2c, 0x10 + (i >> 4));    // Column high
            oled_write_data(i2c, clear_buffer, 32);

            oled_write_cmd(i2c, 0xB0 + page + 1);
            oled_write_cmd(i2c, 0x00 + (i & 0x0F));
            oled_write_cmd(i2c, 0x10 + (i >> 4));
            oled_write_data(i2c, clear_buffer, 32);
        }

        /* Write the string */
        uint8_t col = 0;
        for (uint32_t i = 0; str[i] != '\0' && col < 128; i++) {
            uint16_t font_offset = 4;  // Skip font header
            char c = str[i];
            if (c >= ' ' && c <= '~') {
                font_offset += (c - ' ') * 16;
            } else {
                font_offset += 0 * 16;  // Default to space
            }

            oled_write_cmd(i2c, 0xB0 + page);
            oled_write_cmd(i2c, 0x00 + (col & 0x0F));
            oled_write_cmd(i2c, 0x10 + (col >> 4));
            oled_write_data(i2c, (uint8_t*)&ssd1306xled_font8x16[font_offset], 8);

            oled_write_cmd(i2c, 0xB0 + page + 1);
            oled_write_cmd(i2c, 0x00 + (col & 0x0F));
            oled_write_cmd(i2c, 0x10 + (col >> 4));
            oled_write_data(i2c, (uint8_t*)&ssd1306xled_font8x16[font_offset + 8], 8);

            col += 8;  // 8 pixels per character
        }
        xSemaphoreGive(oled_sem);
        return XST_SUCCESS;
    } else {
        xil_printf("[ERROR] OLED semaphore timeout\r\n");
        return XST_FAILURE;
    }
}

void LCD_Task(void* pvParameters) {
    SensorData_t* sensor_data = (SensorData_t*)pvParameters;
    char buffer[32];

    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        snprintf(buffer, sizeof(buffer), "T:%ld.%02ldC",
                 (long)(sensor_data->temperature / 100), (long)abs(sensor_data->temperature % 100));
        xil_printf("LCD Line 1: %s\r\n", buffer);
        lcd_display_string(&IicInstance, buffer, 0);

        snprintf(buffer, sizeof(buffer), "P:%luh H:%lu%%",
                 (unsigned long)(sensor_data->pressure / 100), (unsigned long)(sensor_data->humidity / 1024));
        xil_printf("LCD Line 2: %s\r\n", buffer);
        lcd_display_string(&IicInstance, buffer, 2);

        snprintf(buffer, sizeof(buffer), "L:%u",
                 sensor_data->luminosity);
        xil_printf("LCD Line 3: %s\r\n", buffer);
        lcd_display_string(&IicInstance, buffer, 4);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
