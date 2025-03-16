/*
 * File: lcd.c
 * Summary:
 * This file implements a driver for the ELEGOO 0.96-inch OLED display (SSD1306 controller)
 * using I2C communication within a FreeRTOS environment. It provides functionality to:
 * - Initialize the OLED display with a sequence of configuration commands
 * - Write commands and data to the display over I2C with semaphore protection
 * - Clear the entire display memory efficiently
 * - Display text strings using an 8x16 font with line buffering to minimize flicker
 * - Run a dedicated task to update the display with sensor data (temperature, pressure,
 *   humidity, and luminosity) in a batched manner at a high refresh rate for smoothness.
 * The driver ensures thread-safe I2C access using FreeRTOS semaphores, reduces I2C traffic
 * through buffering, and prioritizes display updates for a fluid visual experience.
 *
 * Purpose: Provides functions to initialize and display data on the OLED over I2C,
 *          integrated with FreeRTOS for task-based updates with optimized performance.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: [Not specified]
 */

#include "lcd.h"  // Header file with OLED constants (e.g., OLED_I2C_ADDR) and function prototypes

#include <stdio.h>  // For snprintf() to format sensor data strings in LCD_Task

#include "main.h"  // Main project header providing i2c_sem, oled_sem, IicInstance, and sensor_data

extern const uint8_t ssd1306xled_font8x16[];  // External 8x16 font array for rendering characters

static void oled_write_cmd ( XIic* i2c, uint8_t cmd )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 500 ) ) == pdTRUE )
    {
        uint8_t buffer[ 2 ] = { OLED_CMD, cmd };
        int     bytes_sent  = XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP );
        if ( bytes_sent != 2 )
        {
            xil_printf ( "[LCD] Cmd 0x%02X failed, sent %d bytes\n", cmd, bytes_sent );
        }
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[LCD] I2C semaphore timeout in oled_write_cmd\n" );
    }
}

/ static void oled_write_data ( XIic* i2c, uint8_t* data, uint32_t len )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 500 ) ) == pdTRUE )
    {
        uint8_t buffer[ 33 ];
        buffer[ 0 ]            = OLED_DATA;
        uint32_t bytes_to_send = ( len > 32 ) ? 32 : len;
        memcpy ( &buffer[ 1 ], data, bytes_to_send );
        int bytes_sent =
            XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, bytes_to_send + 1, XIIC_STOP );
        if ( bytes_sent != bytes_to_send + 1 )
        {
            xil_printf ( "[LCD] Data write failed, sent %d bytes\n", bytes_sent );
        }
        xSemaphoreGive ( i2c_sem );
    }
    else
    {
        xil_printf ( "[LCD] I2C semaphore timeout in oled_write_data\n" );
    }
}
/*
 * Function: lcd_init
 * Description: Initializes the OLED display by setting its I2C address and sending a sequence
 *              of configuration commands to define display parameters (e.g., resolution, contrast).
 *              Clears the display and turns it on. Uses semaphore protection for initial I2C setup.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: int (XST_SUCCESS if successful, XST_FAILURE if address setup fails)
 */
int lcd_init ( XIic* i2c )
{
    // Take semaphore once for address setup
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 500 ) ) != pdTRUE )
    {
        xil_printf ( "[LCD] Failed to take I2C semaphore\n" );
        return XST_FAILURE;
    }

    int status = XIic_SetAddress ( i2c, XII_ADDR_TO_SEND_TYPE, OLED_I2C_ADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[LCD] Failed to set I2C address 0x%02X: %d\n", OLED_I2C_ADDR, status );
        xSemaphoreGive ( i2c_sem );
        return XST_FAILURE;
    }
    xSemaphoreGive ( i2c_sem );
    xil_printf ( "[LCD] I2C address set to 0x%02X\n", OLED_I2C_ADDR );

    // Original initialization sequence
    oled_write_cmd ( i2c, 0xAE );  // Display off
    oled_write_cmd ( i2c, 0xD5 );
    oled_write_cmd ( i2c, 0x80 );  // Clock settings
    oled_write_cmd ( i2c, 0xA8 );
    oled_write_cmd ( i2c, 0x3F );  // Multiplex ratio
    oled_write_cmd ( i2c, 0xD3 );
    oled_write_cmd ( i2c, 0x00 );  // Offset
    oled_write_cmd ( i2c, 0x40 );  // Start line
    oled_write_cmd ( i2c, 0x8D );
    oled_write_cmd ( i2c, 0x14 );  // Charge pump
    usleep ( 10000 );              // 10ms delay for charge pump
    oled_write_cmd ( i2c, 0x20 );
    oled_write_cmd ( i2c, 0x00 );  // Horizontal addressing
    oled_write_cmd ( i2c, 0xA0 );  // Segment remap
    oled_write_cmd ( i2c, 0xC0 );  // COM scan direction
    oled_write_cmd ( i2c, 0xDA );
    oled_write_cmd ( i2c, 0x12 );  // COM pins
    oled_write_cmd ( i2c, 0x81 );
    oled_write_cmd ( i2c, 0xCF );  // Contrast
    oled_write_cmd ( i2c, 0xD9 );
    oled_write_cmd ( i2c, 0xF1 );  // Pre-charge
    oled_write_cmd ( i2c, 0xDB );
    oled_write_cmd ( i2c, 0x40 );  // VCOMH
    oled_write_cmd ( i2c, 0xA4 );  // Display RAM
    oled_write_cmd ( i2c, 0xA6 );  // Normal mode

    clear_lcd ( i2c );  // Existing clear_lcd function

    oled_write_cmd ( i2c, 0xAF );  // Display on
    usleep ( 10000 );              // 10ms delay for display

    xil_printf ( "[LCD] Initialization complete\n" );
    return XST_SUCCESS;
}

/*
 * Function: clear_lcd
 * Description: Clears the entire OLED display by writing zeros to all 8 pages (128x64 pixels).
 *              Sets page and column addresses before writing 32-byte chunks of zeros.
 *              Used during initialization to ensure a clean starting state.
 * Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 * Returns: None
 */
void clear_lcd ( XIic* i2c )
{
    uint8_t clear_buffer[ 32 ] = { 0 };  // Buffer of 32 zero bytes to clear columns
    for ( uint8_t page = 0; page < 8;
          page++ )  // Iterate over all 8 pages (64 rows / 8 pixels per page)
    {
        oled_write_cmd ( i2c, 0xB0 + page );  // Set page address (0xB0-0xB7 for pages 0-7)
        oled_write_cmd ( i2c, 0x00 );         // Set lower nibble of column address to 0
        oled_write_cmd ( i2c,
                         0x10 );  // Set upper nibble of column address to 0 (start at column 0)
        for ( uint8_t i = 0; i < 128; i += 32 )  // Write 128 columns in 32-byte chunks
        {
            oled_write_data ( i2c, clear_buffer, 32 );  // Clear 32 columns at a time
        }
    }
}

/*
 * Function: lcd_display_string
 * Description: Displays a null-terminated string on the OLED at the specified page using an 8x16
 * font. Each character spans two pages (upper and lower halves). Uses a 256-byte line buffer (128
 * bytes per page) to prepare the entire string locally, then writes it in one pass per page. This
 * reduces flicker and I2C traffic compared to clearing and writing separately. Parameters:
 *   - i2c: Pointer to the XIic instance for I2C communication
 *   - str: Pointer to the null-terminated string to display
 *   - page: Starting page number (0-6, as each character takes 2 pages)
 * Returns: int (XST_SUCCESS if successful, XST_FAILURE if page is invalid)
 */
int lcd_display_string ( XIic* i2c, const char* str, uint8_t page )
{
    if ( page > 7 )
        return XST_FAILURE;  // Validate page number (0-7 max, but 0-6 practical for 2-page chars)

    // Prepare a 256-byte buffer: 128 bytes for the upper page, 128 for the lower page
    uint8_t line_buffer[ 256 ] = { 0 };  // Initialized to zeros to clear unused areas
    uint8_t col                = 0;      // Track current column position (0-127)

    // Build the string in the buffer
    for ( uint32_t i = 0; str[ i ] != '\0' && col < 128;
          i++ )  // Iterate until null or display width reached
    {
        uint16_t font_offset = 4;         // Offset past the 4-byte font header
        char     c           = str[ i ];  // Get the current character
        if ( c >= ' ' && c <= '~' )       // Check if character is in printable ASCII range
        {
            font_offset +=
                ( c - ' ' ) * 16;  // Calculate font array offset (16 bytes per character)
        }
        else
        {
            font_offset += 0 * 16;  // Default to space for non-printable characters
        }

        // Copy the character’s upper and lower 8x8 halves into the buffer
        for ( uint8_t j = 0; j < 8; j++ )  // Each character is 8 pixels wide
        {
            line_buffer[ col + j ] = ssd1306xled_font8x16[ font_offset + j ];  // Upper page data
            line_buffer[ 128 + col + j ] =
                ssd1306xled_font8x16[ font_offset + 8 + j ];  // Lower page data
        }
        col += 8;  // Advance column by 8 pixels (character width)
    }

    // Write the buffered data for the upper page
    oled_write_cmd ( i2c, 0xB0 + page );  // Set the starting page address
    oled_write_cmd ( i2c, 0x00 );         // Set column address lower nibble to 0
    oled_write_cmd ( i2c, 0x10 );  // Set column address upper nibble to 0 (start at column 0)
    for ( uint8_t i = 0; i < 128; i += 32 )  // Write 128 columns in 32-byte chunks
    {
        uint8_t chunk_size = ( 128 - i > 32 ) ? 32 : ( 128 - i );  // Adjust last chunk size if < 32
        oled_write_data ( i2c, &line_buffer[ i ], chunk_size );    // Write upper page data
    }

    // Write the buffered data for the lower page
    oled_write_cmd ( i2c, 0xB0 + page + 1 );  // Set the next page address
    oled_write_cmd ( i2c, 0x00 );             // Reset column address lower nibble to 0
    oled_write_cmd ( i2c, 0x10 );             // Reset column address upper nibble to 0
    for ( uint8_t i = 0; i < 128; i += 32 )   // Write 128 columns in 32-byte chunks
    {
        uint8_t chunk_size = ( 128 - i > 32 ) ? 32 : ( 128 - i );      // Adjust last chunk size
        oled_write_data ( i2c, &line_buffer[ 128 + i ], chunk_size );  // Write lower page data
    }

    return XST_SUCCESS;  // Indicate successful string display
}

/*
 * Function: LCD_Task
 * Description: FreeRTOS task that periodically updates the OLED display with sensor data.
 *              Formats temperature, pressure/humidity, and luminosity into strings and displays
 *              them on pages 0, 2, and 4 respectively. Updates every 100ms (10 Hz) for smooth
 * visuals. Batches all three updates within a single semaphore lock to ensure they appear as one
 * cohesive refresh, minimizing flicker and I2C contention. Parameters:
 *   - pvParameters: Pointer to SensorData_t structure containing sensor readings
 * Returns: None (runs in an infinite loop)
 */
void LCD_Task ( void* pvParameters )
{
    SensorData_t* sensor_data =
        (SensorData_t*) pvParameters;  // Cast parameter to access sensor data
    char buffer[ 32 ];                 // Buffer for formatting sensor data strings (max 32 chars)

    vTaskDelay ( pdMS_TO_TICKS (
        100 ) );  // Initial delay of 100ms to allow system startup before first update

    while ( 1 )  // Infinite loop for continuous display updates
    {
        // Acquire the OLED semaphore with a 100ms timeout to ensure thread-safe access
        if ( xSemaphoreTake ( oled_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
        {
            // Format temperature string (e.g., "T:25.50C")
            snprintf ( buffer,
                       sizeof ( buffer ),
                       "T:%ld.%02ldC",
                       (long) ( sensor_data->temperature / 100 ),  // Integer part of temperature
                       (long) abs ( sensor_data->temperature % 100 ) );  // Decimal part (2 digits)
            lcd_display_string ( &IicInstance, buffer, 0 );  // Update temperature on pages 0-1

            // Format pressure and humidity string (e.g., "P:1013h H:50%")
            snprintf ( buffer,
                       sizeof ( buffer ),
                       "P:%luh H:%lu%%",
                       (unsigned long) ( sensor_data->pressure / 100 ),     // Pressure in hPa
                       (unsigned long) ( sensor_data->humidity / 1024 ) );  // Humidity in percent
            lcd_display_string (
                &IicInstance, buffer, 2 );  // Update pressure/humidity on pages 2-3

            // Format luminosity string (e.g., "L:1234")
            snprintf ( buffer, sizeof ( buffer ), "L:%u", sensor_data->luminosity );  // Lux value
            lcd_display_string ( &IicInstance, buffer, 4 );  // Update luminosity on pages 4-5

            xSemaphoreGive ( oled_sem );  // Release semaphore after all updates are complete
        }
        vTaskDelay ( pdMS_TO_TICKS ( 100 ) );  // Delay 100ms (10 Hz refresh) for smooth updates
    }
}
