/*
 * lcd.c - Driver implementation for ELEGOO 0.96-inch OLED (SSD1306)
 *
 * Purpose: Provides functions to initialize and display data on the ELEGOO
 *          0.96-inch OLED display (SSD1306 controller) via I2C. Integrated with
 *          FreeRTOS for task-based updates, displaying sensor data such as
 *          temperature, pressure, humidity, and luminosity.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Requires Xilinx I2C driver (XIic), FreeRTOS, and font data from
 *               ssd1306xled_font8x16 for character rendering.
 */

#include "lcd.h"

#include <stdio.h> // For snprintf() used in formatting sensor data strings

#include "main.h" // For i2c_sem (I2C semaphore), IicInstance (I2C instance),
                  // and sensor_data (global sensor readings structure)

/*--------------------------------------------------------------------------*/
/* External Declarations                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @brief External font array for 8x16 pixel characters (ASCII 32-126).
 *        Defined elsewhere, typically in a separate font file.
 */
extern const uint8_t ssd1306xled_font8x16[];

/*--------------------------------------------------------------------------*/
/* Static Functions                                                         */
/*--------------------------------------------------------------------------*/
/**
 * oled_write_cmd - Write a single command byte to the OLED via I2C
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @param cmd Command byte to send to the SSD1306 controller
 *
 * Description: Sends a command byte (e.g., display ON/OFF, set page) to the OLED
 *              with the control byte set to OLED_CMD (0x00). Uses a semaphore to
 *              ensure exclusive I2C access in a multi-task environment.
 */
static void oled_write_cmd ( XIic* i2c, uint8_t cmd )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        uint8_t buffer[ 2 ] = { OLED_CMD, cmd }; // Control byte (0x00) + command
        int     status      = XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP );
        if ( status != 2 ) // Expect 2 bytes sent (control + command)
        {
            xil_printf ( "[ERROR] OLED cmd write failed: %d\r\n", status );
        }
        xSemaphoreGive ( i2c_sem ); // Release I2C semaphore
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in oled_write_cmd\r\n" );
    }
}

/**
 * oled_write_data - Write data bytes to the OLED via I2C
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @param data Pointer to the data buffer to send
 * @param len Number of data bytes to send (capped at 32)
 *
 * Description: Sends up to 32 bytes of data (e.g., pixel data) to the OLED with
 *              the control byte set to OLED_DATA (0x40). Uses a semaphore for
 *              I2C access synchronization and limits transactions to 32 bytes
 *              to match SSD1306 buffer constraints.
 */
static void oled_write_data ( XIic* i2c, uint8_t* data, uint32_t len )
{
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        uint8_t buffer[ 33 ];                             // 32 bytes data + 1 control byte
        buffer[ 0 ]            = OLED_DATA;               // Control byte for data (0x40)
        uint32_t bytes_to_send = ( len > 32 ) ? 32 : len; // Cap at 32 bytes
        for ( uint32_t i = 0; i < bytes_to_send; i++ )
        {
            buffer[ i + 1 ] = data[ i ]; // Copy data into buffer after control byte
        }
        int status =
            XIic_Send ( i2c->BaseAddress, OLED_I2C_ADDR, buffer, bytes_to_send + 1, XIIC_STOP );
        if ( status != bytes_to_send + 1 ) // Check exact bytes sent
        {
            xil_printf ( "[ERROR] OLED data write failed: %d\r\n", status );
        }
        xSemaphoreGive ( i2c_sem ); // Release I2C semaphore
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in oled_write_data\r\n" );
    }
}

/*--------------------------------------------------------------------------*/
/* Public Functions                                                         */
/*--------------------------------------------------------------------------*/
/**
 * lcd_init - Initialize the ELEGOO OLED (SSD1306) display
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @return XST_SUCCESS on success, XST_FAILURE on I2C or semaphore failure
 *
 * Description: Sets the OLED I2C address and sends a sequence of initialization
 *              commands to configure the SSD1306 controller (e.g., display settings,
 *              memory addressing mode). Clears the display and turns it on.
 */
int lcd_init ( XIic* i2c )
{
    /* Set the OLED I2C address */
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

    /* SSD1306 initialization sequence (per datasheet) */
    oled_write_cmd ( i2c, 0xAE ); // Display OFF
    oled_write_cmd ( i2c, 0xD5 ); // Set display clock divide ratio/oscillator frequency
    oled_write_cmd ( i2c, 0x80 ); // Default ratio and frequency
    oled_write_cmd ( i2c, 0xA8 ); // Set multiplex ratio
    oled_write_cmd ( i2c, 0x3F ); // 64 MUX (for 128x64 display)
    oled_write_cmd ( i2c, 0xD3 ); // Set display offset
    oled_write_cmd ( i2c, 0x00 ); // No offset
    oled_write_cmd ( i2c, 0x40 ); // Set display start line to 0
    oled_write_cmd ( i2c, 0x8D ); // Charge pump setting
    oled_write_cmd ( i2c, 0x14 ); // Enable charge pump
    oled_write_cmd ( i2c, 0x20 ); // Set memory addressing mode
    oled_write_cmd ( i2c, 0x00 ); // Horizontal addressing mode
    oled_write_cmd ( i2c, 0xA0 ); // Segment remap (no horizontal flip)
    oled_write_cmd ( i2c, 0xC0 ); // COM output scan direction (no vertical flip)
    oled_write_cmd ( i2c, 0xDA ); // Set COM pins hardware configuration
    oled_write_cmd ( i2c, 0x12 ); // Alternative COM pin config for 128x64
    oled_write_cmd ( i2c, 0x81 ); // Set contrast control
    oled_write_cmd ( i2c, 0xCF ); // Contrast value (moderate brightness)
    oled_write_cmd ( i2c, 0xD9 ); // Set pre-charge period
    oled_write_cmd ( i2c, 0xF1 ); // Phase 1: 1, Phase 2: 15
    oled_write_cmd ( i2c, 0xDB ); // Set VCOMH deselect level
    oled_write_cmd ( i2c, 0x40 ); // VCOMH level ~0.83 * VCC
    oled_write_cmd ( i2c, 0xA4 ); // Entire display on (normal mode)
    oled_write_cmd ( i2c, 0xA6 ); // Normal display (not inverted)

    /* Clear the entire 128x64 display (8 pages, 128 columns) */
    uint8_t clear_buffer[ 32 ] = { 0 }; // Buffer of zeros for clearing
    for ( uint8_t page = 0; page < 8; page++ )
    {
        oled_write_cmd ( i2c, 0xB0 + page );    // Set page address (0-7)
        oled_write_cmd ( i2c, 0x00 );           // Set column lower nibble to 0
        oled_write_cmd ( i2c, 0x10 );           // Set column upper nibble to 0
        for ( uint8_t i = 0; i < 128; i += 32 ) // Clear 128 columns in 32-byte chunks
        {
            oled_write_data ( i2c, clear_buffer, 32 );
        }
    }

    oled_write_cmd ( i2c, 0xAF ); // Display ON
    return XST_SUCCESS;
}

/**
 * lcd_display_string - Display a string on the OLED at a specified page
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @param str Null-terminated string to display
 * @param page Starting page number (0-7) for the 16-pixel tall characters
 * @return XST_SUCCESS on success, XST_FAILURE on invalid page or semaphore timeout
 *
 * Description: Displays a string using an 8x16 font on two consecutive pages
 *              (16 pixels tall). Clears the target area first, then renders each
 *              character from the font array. Limits display to 128 columns.
 */
int lcd_display_string ( XIic* i2c, const char* str, uint8_t page )
{
    if ( page > 7 ) // Validate page number (0-7 for 128x64 display)
        return XST_FAILURE;

    if ( xSemaphoreTake ( oled_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        /* Clear the two pages (16 pixels tall) where the string will be displayed */
        uint8_t clear_buffer[ 32 ] = { 0 };     // Buffer of zeros for clearing
        for ( uint8_t i = 0; i < 128; i += 32 ) // Clear 128 columns in 32-byte chunks
        {
            oled_write_cmd ( i2c, 0xB0 + page );         // Set first page
            oled_write_cmd ( i2c, 0x00 + ( i & 0x0F ) ); // Column lower nibble
            oled_write_cmd ( i2c, 0x10 + ( i >> 4 ) );   // Column upper nibble
            oled_write_data ( i2c, clear_buffer, 32 );

            oled_write_cmd ( i2c, 0xB0 + page + 1 ); // Set second page
            oled_write_cmd ( i2c, 0x00 + ( i & 0x0F ) );
            oled_write_cmd ( i2c, 0x10 + ( i >> 4 ) );
            oled_write_data ( i2c, clear_buffer, 32 );
        }

        /* Render the string character by character */
        uint8_t col = 0; // Current column position (0-127)
        for ( uint32_t i = 0; str[ i ] != '\0' && col < 128; i++ )
        {
            uint16_t font_offset = 4; // Skip 4-byte font header
            char     c           = str[ i ];
            if ( c >= ' ' && c <= '~' ) // Printable ASCII range
            {
                font_offset += ( c - ' ' ) * 16; // 16 bytes per character
            }
            else
            {
                font_offset += 0 * 16; // Default to space for non-printable chars
            }

            /* Write top 8 pixels of character */
            oled_write_cmd ( i2c, 0xB0 + page );           // Set first page
            oled_write_cmd ( i2c, 0x00 + ( col & 0x0F ) ); // Column lower nibble
            oled_write_cmd ( i2c, 0x10 + ( col >> 4 ) );   // Column upper nibble
            oled_write_data ( i2c, (uint8_t*) &ssd1306xled_font8x16[ font_offset ], 8 );

            /* Write bottom 8 pixels of character */
            oled_write_cmd ( i2c, 0xB0 + page + 1 ); // Set second page
            oled_write_cmd ( i2c, 0x00 + ( col & 0x0F ) );
            oled_write_cmd ( i2c, 0x10 + ( col >> 4 ) );
            oled_write_data ( i2c, (uint8_t*) &ssd1306xled_font8x16[ font_offset + 8 ], 8 );

            col += 8; // Advance 8 pixels (character width)
        }
        xSemaphoreGive ( oled_sem ); // Release OLED semaphore
        return XST_SUCCESS;
    }
    else
    {
        xil_printf ( "[ERROR] OLED semaphore timeout\r\n" );
        return XST_FAILURE;
    }
}

/**
 * LCD_Task - FreeRTOS task to update the OLED with sensor data
 *
 * @param pvParameters Pointer to SensorData_t structure containing sensor readings
 *
 * Description: Periodically formats and displays temperature, pressure/humidity,
 *              and luminosity data on the OLED at pages 0, 2, and 4 respectively.
 *              Updates every 1 second using FreeRTOS task delay.
 */
void LCD_Task ( void* pvParameters )
{
    SensorData_t* sensor_data = (SensorData_t*) pvParameters; // Cast parameter to sensor data
    char          buffer[ 32 ]; // Buffer for formatted strings (max 32 chars)

    vTaskDelay ( pdMS_TO_TICKS ( 100 ) ); // Initial delay of 100ms for system stabilization

    while ( 1 )
    {
        /* Format and display temperature (e.g., "T:25.30C") */
        snprintf ( buffer,
                   sizeof ( buffer ),
                   "T:%ld.%02ldC",
                   (long) ( sensor_data->temperature / 100 ),       // Integer part (°C)
                   (long) abs ( sensor_data->temperature % 100 ) ); // Decimal part (°C)
        xil_printf ( "LCD Line 1: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 0 ); // Display on page 0

        /* Format and display pressure/humidity (e.g., "P:1013h H:50%") */
        snprintf ( buffer,
                   sizeof ( buffer ),
                   "P:%luh H:%lu%%",
                   (unsigned long) ( sensor_data->pressure / 100 ),    // Pressure in hPa
                   (unsigned long) ( sensor_data->humidity / 1024 ) ); // Humidity in %
        xil_printf ( "LCD Line 2: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 2 ); // Display on page 2

        /* Format and display luminosity (e.g., "L:1234") */
        snprintf ( buffer, sizeof ( buffer ), "L:%u", sensor_data->luminosity );
        xil_printf ( "LCD Line 3: %s\r\n", buffer );
        lcd_display_string ( &IicInstance, buffer, 4 ); // Display on page 4

        vTaskDelay ( pdMS_TO_TICKS ( 1000 ) ); // Wait 1 second before next update
    }
}