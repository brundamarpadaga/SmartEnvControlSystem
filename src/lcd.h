/*
 * File: lcd.h
 * Summary:
 * Header file for the ELEGOO 0.96-inch OLED display (SSD1306 controller) driver.
 * Defines constants, external variables, and function prototypes for initializing
 * and controlling the OLED display over I2C within a FreeRTOS environment. The driver
 * supports displaying sensor data (temperature, pressure, humidity, luminosity) and
 * clearing the display, with thread-safe access ensured by semaphores.
 *
 * Purpose: Provides the interface for OLED display operations, including initialization,
 *          string display, display clearing, and a dedicated FreeRTOS task for updates.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: [Not specified]
 */

#ifndef LCD_H
#define LCD_H

#include "FreeRTOS.h"  // For FreeRTOS types and functions (e.g., SemaphoreHandle_t, pdMS_TO_TICKS)
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"      // For semaphore handling
#include "task.h"        // For task management (e.g., vTaskDelay)
#include "xiic.h"        // For Xilinx I2C driver (XIic type and functions)
#include "xil_printf.h"  // For xil_printf debugging output

/*
 * Constants:
 * - OLED_I2C_ADDR: I2C address of the SSD1306 OLED controller (0x3C)
 * - OLED_CMD: Control byte for sending commands to the OLED (0x00)
 * - OLED_DATA: Control byte for sending data to the OLED (0x40)
 */
#define OLED_I2C_ADDR 0x3C  // Default I2C address for SSD1306 OLED
#define OLED_CMD 0x00       // Command mode identifier for I2C transactions
#define OLED_DATA 0x40      // Data mode identifier for I2C transactions

/*
 * External Declarations:
 * - IicInstance: Global I2C instance defined in main.c, used for communication
 * - oled_sem: Semaphore for synchronizing access to the OLED display
 * - i2c_sem: Semaphore for synchronizing access to the I2C bus
 * - ssd1306xled_font8x16: Font array (8x16 pixels) for character rendering
 */
extern XIic              IicInstance;             // I2C instance for OLED communication
extern SemaphoreHandle_t oled_sem;                // Semaphore for OLED access control
extern SemaphoreHandle_t i2c_sem;                 // Semaphore for I2C bus access control
extern const uint8_t     ssd1306xled_font8x16[];  // 8x16 font data for text display

/*
 * Function: lcd_init
 * Description: Initializes the OLED display by configuring the I2C address and sending
 *              a sequence of commands to set up display parameters. Clears the display
 *              and turns it on. Uses semaphore protection for thread-safe I2C access.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: int (XST_SUCCESS on success, XST_FAILURE on failure)
 */
int lcd_init ( XIic* i2c );

void lcd_display_string_via_queue ( XIic*        i2c,
                                    const char*  str,
                                    uint8_t      page,
                                    xQueueHandle reply_queue );

/*
 * Function: LCD_Task
 * Description: FreeRTOS task that periodically updates the OLED display with sensor data.
 *              Formats temperature, pressure, humidity, and luminosity into strings and
 *              displays them on separate lines (pages 0, 2, 4) every 500ms.
 * Parameters:
 *   - pvParameters: Pointer to SensorData_t structure containing sensor readings
 * Returns: None (infinite loop)
 */
void LCD_Task ( void* pvParameters );

/*
 * Function: clear_lcd
 * Description: Clears the entire OLED display by writing zeros to all 8 pages (128x64 pixels).
 *              Sets column and page addresses before each write operation.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: None
 */
void clear_lcd ( XIic* i2c );

#endif /* LCD_H */
