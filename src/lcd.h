#ifndef LCD_H // Include guard to prevent multiple inclusions
#define LCD_H

/*
 * lcd.h - Header file for ELEGOO 0.96-inch OLED (SSD1306) driver
 *
 * Purpose: Defines constants, global variables, and function prototypes for
 *          initializing and managing the ELEGOO 0.96-inch OLED display with
 *          an SSD1306 controller. Designed for use in a FreeRTOS-based embedded
 *          system on a Microblaze platform, interfacing via I2C to display sensor data.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Relies on Xilinx BSP headers (xiic.h, xil_printf.h) for I2C and
 *               debug output, and FreeRTOS headers for task and semaphore management.
 */

/*--------------------------------------------------------------------------*/
/* Header Includes                                                         */
/*--------------------------------------------------------------------------*/
#include "FreeRTOS.h"   // FreeRTOS kernel definitions for task scheduling and timing
#include "semphr.h"     // Semaphore definitions for FreeRTOS (e.g., SemaphoreHandle_t)
#include "task.h"       // Task management APIs (e.g., vTaskDelay)
#include "xiic.h"       // Xilinx I2C driver API for communication with the SSD1306
#include "xil_printf.h" // Xilinx lightweight printf for debug and status output

/*--------------------------------------------------------------------------*/
/* Macro Definitions                                                        */
/*--------------------------------------------------------------------------*/
/**
 * @def OLED_I2C_ADDR
 * @brief I2C address of the ELEGOO OLED (SSD1306), typically 0x3C.
 *        Determined by the SA0 pin state (0x3C if SA0 is low, 0x3D if high).
 */
#define OLED_I2C_ADDR 0x3C

/**
 * @def OLED_CMD
 * @brief Control byte for sending commands to the SSD1306 (0x00).
 *        Indicates the following byte is a command (e.g., set page, display ON).
 */
#define OLED_CMD 0x00

/**
 * @def OLED_DATA
 * @brief Control byte for sending data to the SSD1306 (0x40).
 *        Indicates the following bytes are pixel data for the display RAM.
 */
#define OLED_DATA 0x40

/*--------------------------------------------------------------------------*/
/* External Declarations                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @brief Global I2C instance for communication with the OLED.
 *        Defined in main.c or another source file and used by all LCD functions.
 */
extern XIic IicInstance;

/**
 * @brief Semaphore for synchronizing access to the OLED display in FreeRTOS.
 *        Ensures exclusive access to the display across multiple tasks.
 */
extern SemaphoreHandle_t oled_sem;

/**
 * @brief Semaphore for synchronizing I2C bus access in FreeRTOS.
 *        Prevents concurrent I2C transactions from multiple peripherals.
 */
extern SemaphoreHandle_t i2c_sem;

/**
 * @brief Font array for 8x16 pixel characters (ASCII 32-126).
 *        Defined externally (e.g., in a font file) and used for string rendering.
 */
extern const uint8_t ssd1306xled_font8x16[];

/*--------------------------------------------------------------------------*/
/* Function Prototypes                                                      */
/*--------------------------------------------------------------------------*/
/**
 * @brief Initialize the ELEGOO OLED (SSD1306) display.
 * @param i2c Pointer to the XIic instance for I2C communication
 * @return XST_SUCCESS on success, XST_FAILURE on I2C or semaphore failure
 *
 * Description: Configures the SSD1306 controller via I2C with initialization
 *              commands (e.g., display settings, memory mode), clears the display,
 *              and turns it on. Sets the OLED I2C address to OLED_I2C_ADDR.
 */
int lcd_init ( XIic* i2c );

/**
 * @brief Display a string on the OLED at a specified page.
 * @param i2c Pointer to the XIic instance for I2C communication
 * @param str Null-terminated string to display
 * @param page Starting page number (0-7) for 16-pixel tall characters
 * @return XST_SUCCESS on success, XST_FAILURE on invalid page or semaphore timeout
 *
 * Description: Renders a string on the OLED using an 8x16 font across two pages
 *              (16 pixels tall). Clears the target area before writing characters,
 *              with a maximum width of 128 columns.
 */
int lcd_display_string ( XIic* i2c, const char* str, uint8_t page );

/**
 * @brief FreeRTOS task to periodically update the OLED with sensor data.
 * @param pvParameters Pointer to SensorData_t structure containing sensor readings
 *
 * Description: Formats and displays temperature, pressure/humidity, and luminosity
 *              data on the OLED at pages 0, 2, and 4, respectively. Updates every
 *              1 second using FreeRTOS task delay.
 */
void LCD_Task ( void* pvParameters );

#endif /* LCD_H */