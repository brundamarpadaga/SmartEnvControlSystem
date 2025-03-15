/*
 * File: lcd.c
 * Summary:
 * This file implements a driver for the ELEGOO 0.96-inch OLED display (SSD1306 controller)
 * using I2C communication within a FreeRTOS environment. It provides functionality to:
 * - Initialize the OLED display with appropriate configuration commands
 * - Write commands and data to the display over I2C with semaphore protection
 * - Clear the display memory
 * - Display text strings using an 8x16 font
 * - Run a dedicated task to periodically update the display with sensor data
 * (temperature, pressure, humidity, and luminosity). The driver ensures thread-safe
 * access to the I2C bus and OLED hardware using FreeRTOS semaphores.
 *
 * Purpose: Provides functions to initialize and display data on the OLED over I2C,
 *          integrated with FreeRTOS for task-based updates.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: [Not specified]
 */

#include "lcd.h"  // Header for OLED constants and function prototypes

#include <stdio.h>  // For snprintf in LCD_Task

#include "main.h"  // For i2c_sem, IicInstance, sensor_data

extern const uint8_t ssd1306xled_font8x16[];  // External font array

// Function prototypes
static void oled_write_cmd ( XIic* i2c, uint8_t cmd );                   // Write command to OLED
static void oled_write_data ( XIic* i2c, uint8_t* data, uint32_t len );  // Write data to OLED
void        clear_lcd ( XIic* i2c );                                     // Clear OLED display

/*
 * Function: oled_write_cmd
 * Description: Sends a single command byte to the OLED display over I2C.
 *              Uses a semaphore to ensure exclusive access to the I2C bus.
 *              Reports errors if the semaphore cannot be taken or if the write fails.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 *   - cmd: Command byte to send to the OLED
 * Returns: None
 */
static void oled_write_cmd ( XIic* i2c, uint8_t cmd )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        uint8_t buffer[ 2 ] = { OLED_CMD, cmd };  // Buffer: control byte + command
        int     status      = XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP );
        if ( status != 2 )
        {
            xil_printf ( "[ERROR] OLED cmd write failed: %d\r\n", status );
        }
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in oled_write_cmd\r\n" );
    }
}

/*
 * Function: oled_write_data
 * Description: Writes a block of data (up to 32 bytes) to the OLED display over I2C.
 *              Uses a semaphore for thread-safe I2C access. Limits data to 32 bytes
 *              per transaction to comply with OLED hardware constraints.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 *   - data: Pointer to the data buffer to send
 *   - len: Length of data to send (capped at 32 bytes)
 * Returns: None
 */
static void oled_write_data ( XIic* i2c, uint8_t* data, uint32_t len )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        uint8_t buffer[ 33 ];  // Buffer: 1 control byte + up to 32 data bytes
        buffer[ 0 ]            = OLED_DATA;
        uint32_t bytes_to_send = ( len > 32 ) ? 32 : len;
        for ( uint32_t i = 0; i < bytes_to_send; i++ )
        {
            buffer[ i + 1 ] = data[ i ];
        }
        int status =
            XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, bytes_to_send + 1, XIIC_STOP );
        if ( status != bytes_to_send + 1 )
        {
            xil_printf ( "[ERROR] OLED data write failed: %d\r\n", status );
        }
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in oled_write_data\r\n" );
    }
}

/*
 * Function: lcd_init
 * Description: Initializes the OLED display by setting the I2C address and sending
 *              a sequence of configuration commands to set up the display parameters
 *              (e.g., resolution, contrast, memory addressing). Clears the display
 *              and turns it on. Uses semaphore protection for I2C access.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: int (XST_SUCCESS on success, XST_FAILURE on failure)
 */
int lcd_init ( XIic* i2c )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        int status = XIic_SetAddress ( i2c, XII_ADDR_TO_SEND_TYPE, OLED_I2C_ADDR );
        if ( status != XST_SUCCESS )
        {
            xil_printf ( "[ERROR] Failed to set OLED I2C address\r\n" );
            xSemaphoreGive ( i2c_sem );
            return XST_FAILURE;
        }
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in lcd_init\r\n" );
        return XST_FAILURE;
    }

    oled_write_cmd ( i2c, 0xAE );  // Display OFF
    oled_write_cmd ( i2c, 0xD5 );  // Set clock divide ratio/oscillator frequency
    oled_write_cmd ( i2c, 0x80 );  // Default clock settings
    oled_write_cmd ( i2c, 0xA8 );  // Set multiplex ratio
    oled_write_cmd ( i2c, 0x3F );  // 64 MUX (128x64)
    oled_write_cmd ( i2c, 0xD3 );  // Set display offset
    oled_write_cmd ( i2c, 0x00 );  // No offset
    oled_write_cmd ( i2c, 0x40 );  // Set start line to 0
    oled_write_cmd ( i2c, 0x8D );  // Charge pump setting
    oled_write_cmd ( i2c, 0x14 );  // Enable charge pump
    oled_write_cmd ( i2c, 0x20 );  // Set memory addressing mode
    oled_write_cmd ( i2c, 0x00 );  // Horizontal addressing
    oled_write_cmd ( i2c, 0xA0 );  // Segment remap
    oled_write_cmd ( i2c, 0xC0 );  // COM output scan direction
    oled_write_cmd ( i2c, 0xDA );  // Set COM pins config
    oled_write_cmd ( i2c, 0x12 );  // Alternative COM config
    oled_write_cmd ( i2c, 0x81 );  // Set contrast control
    oled_write_cmd ( i2c, 0xCF );  // High contrast
    oled_write_cmd ( i2c, 0xD9 );  // Set pre-charge period
    oled_write_cmd ( i2c, 0xF1 );  // Phase 1: 1, Phase 2: 15
    oled_write_cmd ( i2c, 0xDB );  // Set VCOMH deselect level
    oled_write_cmd ( i2c, 0x40 );  // VCOMH level
    oled_write_cmd ( i2c, 0xA4 );  // Resume RAM content
    oled_write_cmd ( i2c, 0xA6 );  // Normal display

    clear_lcd ( i2c );  // Clear display memory

    oled_write_cmd ( i2c, 0xAF );  // Display ON
    return XST_SUCCESS;
}

/*
 * Function: clear_lcd
 * Description: Clears the entire OLED display by writing zeros to all 8 pages
 *              (128x64 pixels). Sets the column and page addresses before each write.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: None
 */
void clear_lcd ( XIic* i2c )
{
    uint8_t clear_buffer[ 32 ] = { 0 };
    for ( uint8_t page = 0; page < 8; page++ )
    {
        oled_write_cmd ( i2c, 0xB0 + page );  // Set page address
        oled_write_cmd ( i2c, 0x00 );         // Lower column address
        oled_write_cmd ( i2c, 0x10 );         // Higher column address
        for ( uint8_t i = 0; i < 128; i += 32 )
        {
            oled_write_data ( i2c, clear_buffer, 32 );
        }
    }
}

/*
 * Function: lcd_display_string
 * Description: Displays a null-terminated string on the OLED at the specified page.
 *              Uses an 8x16 font, writing each character across two pages (upper and lower halves).
 *              Ensures thread-safe access with a semaphore.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 *   - str: Pointer to the null-terminated string to display
 *   - page: Starting page number (0-6, as each char takes 2 pages)
 * Returns: int (XST_SUCCESS on success, XST_FAILURE on failure)
 */
int lcd_display_string ( XIic* i2c, const char* str, uint8_t page )
{
    if ( page > 7 ) return XST_FAILURE;

    if ( xSemaphoreTake ( oled_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
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
        uint8_t col = 0;
        for ( uint32_t i = 0; str[ i ] != '\0' && col < 128; i++ )
        {
            uint16_t font_offset = 4;
            char     c           = str[ i ];
            if ( c >= ' ' && c <= '~' )
            {
                font_offset += ( c - ' ' ) * 16;
            }
            else
            {
                font_offset += 0 * 16;
            }

            oled_write_cmd ( i2c, 0xB0 + page );
            oled_write_cmd ( i2c, 0x00 + ( col & 0x0F ) );
            oled_write_cmd ( i2c, 0x10 + ( col >> 4 ) );
            oled_write_data ( i2c, (uint8_t*) &ssd1306xled_font8x16[ font_offset ], 8 );

            oled_write_cmd ( i2c, 0xB0 + page + 1 );
            oled_write_cmd ( i2c, 0x00 + ( col & 0x0F ) );
            oled_write_cmd ( i2c, 0x10 + ( col >> 4 ) );
            oled_write_data ( i2c, (uint8_t*) &ssd1306xled_font8x16[ font_offset + 8 ], 8 );

            col += 8;
        }
        xSemaphoreGive ( oled_sem );
        return XST_SUCCESS;
    }
    else
    {
        xil_printf ( "[ERROR] OLED semaphore timeout\r\n" );
        return XST_FAILURE;
    }
}

/*
 * Function: LCD_Task
 * Description: FreeRTOS task that periodically updates the OLED display with sensor data.
 *              Formats temperature, pressure, humidity, and luminosity into strings and
 *              displays them on separate lines (pages 0, 2, 4). Updates every 500ms.
 * Parameters:
 *   - pvParameters: Pointer to SensorData_t structure containing sensor readings
 * Returns: None (infinite loop)
 */
void LCD_Task ( void* pvParameters )
{
    SensorData_t* sensor_data = (SensorData_t*) pvParameters;
    char          buffer[ 32 ];

    vTaskDelay ( pdMS_TO_TICKS ( 500 ) );

    while ( 1 )
    {
        snprintf ( buffer,
                   sizeof ( buffer ),
                   "T:%ld.%02ldC",
                   (long) ( sensor_data->temperature / 100 ),
                   (long) abs ( sensor_data->temperature % 100 ) );
        xil_printf ( "LCD Line 1: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 0 );

        snprintf ( buffer,
                   sizeof ( buffer ),
                   "P:%luh H:%lu%%",
                   (unsigned long) ( sensor_data->pressure / 100 ),
                   (unsigned long) ( sensor_data->humidity / 1024 ) );
        xil_printf ( "LCD Line 2: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 2 );

        snprintf ( buffer, sizeof ( buffer ), "L:%u", sensor_data->luminosity );
        xil_printf ( "LCD Line 3: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 4 );

        vTaskDelay ( pdMS_TO_TICKS ( 500 ) );
    }
}
