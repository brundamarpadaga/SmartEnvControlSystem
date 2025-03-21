/*
 * ===========================================================================
 * File:    lcd.c
 * Summary: OLED SSD1306 Display Driver for Environmental Control System
 * ===========================================================================
 * Purpose: This file provides functions to interface with an SSD1306 OLED
 *          display over I2C in a FreeRTOS environment. It includes initialization,
 *          clearing, and string display capabilities using an 8x16 font. The
 *          LCD task periodically updates the display with sensor data (temperature,
 *          humidity, pressure, and luminosity) from the shared sensor_Data structure.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 *
 *
 * Dependencies:
 *   - lcd.h: Header file defining function prototypes and constants
 *   - main.h: Provides I2C instance, semaphores, and queue definitions
 *   - FreeRTOS: Task management, semaphores, and queues
 *   - XIic: Xilinx I2C driver for low-level communication
 *
 * Notes:
 *   - Uses a static 256-byte line buffer to store display data across calls.
 *   - Font data (ssd1306xled_font8x16) supports ASCII characters from space (0x20)
 *     to tilde (0x7E) in an 8x16 pixel format.
 *   - All I2C operations are protected by semaphores for thread safety.
 */

/* ===========================================================================
 * Includes
 * ===========================================================================
 */
#include "lcd.h"    // OLED-specific definitions and prototypes
#include "main.h"   // System-wide definitions (I2C instance, semaphores, etc.)
#include <stdio.h>  // For snprintf in LCD_Task
#include <string.h> // For memset in lcd_display_string_via_queue

/* ===========================================================================
 * External Declarations
 * ===========================================================================
 */
extern const uint8_t ssd1306xled_font8x16[]; // Font data defined at file end

/* ===========================================================================
 * Global Variables
 * ===========================================================================
 */
static uint8_t line_buffer[256]; // Static buffer for display data, persists across calls

/* ===========================================================================
 * Private Function Definitions
 * ===========================================================================
 */

/**
 * @brief Writes a single command byte to the OLED display via I2C.
 * @details Acquires the I2C semaphore, sends the command prefixed with OLED_CMD,
 *          and releases the semaphore. Uses a 100ms timeout for semaphore access.
 * @param i2c Pointer to the I2C instance.
 * @param cmd Command byte to send (e.g., display on/off, page address).
 */
static void oled_write_cmd(XIic* i2c, uint8_t cmd)
{
    // Attempt to acquire I2C bus semaphore with 100ms timeout
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        uint8_t buffer[2] = {OLED_CMD, cmd}; // Buffer: control byte + command
        // Send 2 bytes to OLED I2C address with stop condition
        XIic_Send(i2c->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
        xSemaphoreGive(i2c_sem); // Release I2C bus
    }
}

/**
 * @brief Writes data bytes to the OLED display via I2C.
 * @details Acquires the I2C semaphore, sends up to 32 bytes of data prefixed with
 *          OLED_DATA, and releases the semaphore. Limits transfer to 32 bytes.
 * @param i2c Pointer to the I2C instance.
 * @param data Pointer to the data buffer to send.
 * @param len Number of bytes to send (capped at 32).
 */
static void oled_write_data(XIic* i2c, uint8_t* data, uint32_t len)
{
    // Attempt to acquire I2C bus semaphore with 100ms timeout
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        uint8_t buffer[33];    // Buffer: control byte + up to 32 data bytes
        buffer[0] = OLED_DATA; // Set control byte for data
        // Determine bytes to send, capped at 32
        uint32_t bytes_to_send = (len > 32) ? 32 : len;
        // Copy data into buffer after control byte
        for (uint32_t i = 0; i < bytes_to_send; i++)
        {
            buffer[i + 1] = data[i];
        }
        // Send buffer to OLED I2C address with stop condition
        XIic_Send(i2c->BaseAddress, OLED_I2C_ADDR, buffer, bytes_to_send + 1, XIIC_STOP);
        xSemaphoreGive(i2c_sem); // Release I2C bus
    }
}

/* ===========================================================================
 * Public Function Definitions
 * ===========================================================================
 */

/**
 * @brief Initializes the SSD1306 OLED display.
 * @details Sets the I2C address and sends a sequence of commands to configure
 *          the display (e.g., multiplex ratio, contrast, memory mode). Clears
 *          the display and turns it on.
 * @param i2c Pointer to the I2C instance.
 * @return int XST_SUCCESS on success, XST_FAILURE on I2C setup failure.
 */
int lcd_init(XIic* i2c)
{
    // Attempt to acquire I2C bus semaphore with 100ms timeout
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // Set OLED I2C address for communication
        int status = XIic_SetAddress(i2c, XII_ADDR_TO_SEND_TYPE, OLED_I2C_ADDR);
        if (status != XST_SUCCESS)
        {
            xSemaphoreGive(i2c_sem); // Release I2C bus on failure
            return XST_FAILURE;
        }
        xSemaphoreGive(i2c_sem); // Release I2C bus
    }
    else
    {
        return XST_FAILURE; // Return failure if semaphore not acquired
    }

    // Send initialization commands to SSD1306
    oled_write_cmd(i2c, 0xAE); // Display off
    oled_write_cmd(i2c, 0xD5); // Set display clock divide ratio/oscillator frequency
    oled_write_cmd(i2c, 0x80); // Default value for clock
    oled_write_cmd(i2c, 0xA8); // Set multiplex ratio
    oled_write_cmd(i2c, 0x3F); // 64 MUX (for 128x64 display)
    oled_write_cmd(i2c, 0xD3); // Set display offset
    oled_write_cmd(i2c, 0x00); // No offset
    oled_write_cmd(i2c, 0x40); // Set start line to 0
    oled_write_cmd(i2c, 0x8D); // Charge pump setting
    oled_write_cmd(i2c, 0x14); // Enable charge pump
    oled_write_cmd(i2c, 0x20); // Set memory addressing mode
    oled_write_cmd(i2c, 0x00); // Horizontal addressing mode
    oled_write_cmd(i2c, 0xA0); // Segment remap (column address 0 mapped to SEG0)
    oled_write_cmd(i2c, 0xC0); // COM output scan direction (normal)
    oled_write_cmd(i2c, 0xDA); // Set COM pins hardware configuration
    oled_write_cmd(i2c, 0x12); // Alternative COM pin config
    oled_write_cmd(i2c, 0x81); // Set contrast control
    oled_write_cmd(i2c, 0xCF); // Contrast value (default: high)
    oled_write_cmd(i2c, 0xD9); // Set pre-charge period
    oled_write_cmd(i2c, 0xF1); // Phase 1: 1, Phase 2: 15
    oled_write_cmd(i2c, 0xDB); // Set VCOMH deselect level
    oled_write_cmd(i2c, 0x40); // ~0.77 * Vcc
    oled_write_cmd(i2c, 0xA4); // Entire display on (resume RAM content)
    oled_write_cmd(i2c, 0xA6); // Normal display (not inverted)

    clear_lcd(i2c);            // Clear display RAM
    oled_write_cmd(i2c, 0xAF); // Turn display on
    return XST_SUCCESS;        // Return success
}

/**
 * @brief Clears the OLED display.
 * @details Sets all pixels to off by writing zeros to each page (0-7) of the
 *          display RAM, 128 columns at a time.
 * @param i2c Pointer to the I2C instance.
 */
void clear_lcd(XIic* i2c)
{
    uint8_t clear_buffer[32] = {0}; // Buffer of zeros for clearing
    // Iterate through all 8 pages (128x64 display has 8 pages of 8 bits each)
    for (uint8_t page = 0; page < 8; page++)
    {
        oled_write_cmd(i2c, 0xB0 + page); // Set page address
        oled_write_cmd(i2c, 0x00);        // Set column start address low nibble
        oled_write_cmd(i2c, 0x10);        // Set column start address high nibble
        // Write 128 columns in chunks of 32
        for (uint8_t i = 0; i < 128; i += 32)
        {
            oled_write_data(i2c, clear_buffer, 32);
        }
    }
}

/**
 * @brief Displays a string on the OLED at a specified page.
 * @details Converts the string to font data (8x16 pixels per character) and
 *          writes it to two consecutive pages (upper and lower halves).
 * @param i2c Pointer to the I2C instance.
 * @param str Null-terminated string to display.
 * @param page Starting page (0-6, as it uses 2 pages).
 * @return int XST_SUCCESS on success, XST_FAILURE if page invalid.
 */
int lcd_display_string(XIic* i2c, const char* str, uint8_t page)
{
    if (page > 7)
        return XST_FAILURE; // Validate page (0-7, but needs 2 pages)

    uint8_t line_buffer[256] = {0}; // Local buffer for font data
    uint8_t col              = 0;   // Current column position

    // Convert string to font data
    for (uint32_t i = 0; str[i] != '\0' && col < 128; i++)
    {
        uint16_t font_offset = 4; // Base offset in font array
        char     c           = str[i];
        // Map printable ASCII characters to font indices
        if (c >= ' ' && c <= '~')
        {
            font_offset += (c - ' ') * 16; // 16 bytes per character
        }
        // Copy 8 columns for upper and lower halves
        for (uint8_t j = 0; j < 8; j++)
        {
            line_buffer[col + j]       = ssd1306xled_font8x16[font_offset + j];     // Upper 8 bits
            line_buffer[128 + col + j] = ssd1306xled_font8x16[font_offset + 8 + j]; // Lower 8 bits
        }
        col += 8; // Advance column by character width
    }

    // Write upper page
    oled_write_cmd(i2c, 0xB0 + page); // Set page address
    oled_write_cmd(i2c, 0x00);        // Column start low nibble
    oled_write_cmd(i2c, 0x10);        // Column start high nibble
    for (uint8_t i = 0; i < 128; i += 32)
    {
        uint8_t chunk_size = (128 - i > 32) ? 32 : (128 - i); // Adjust for last chunk
        oled_write_data(i2c, &line_buffer[i], chunk_size);
    }

    // Write lower page
    oled_write_cmd(i2c, 0xB0 + page + 1); // Next page
    oled_write_cmd(i2c, 0x00);            // Column start low nibble
    oled_write_cmd(i2c, 0x10);            // Column start high nibble
    for (uint8_t i = 0; i < 128; i += 32)
    {
        uint8_t chunk_size = (128 - i > 32) ? 32 : (128 - i); // Adjust for last chunk
        oled_write_data(i2c, &line_buffer[128 + i], chunk_size);
    }

    return XST_SUCCESS;
}

/**
 * @brief FreeRTOS task to update OLED with sensor data.
 * @details Periodically displays temperature, humidity, pressure, and luminosity
 *          on pages 0, 2, 4, and 6 respectively, using a queue for I2C replies.
 * @param pvParameters Pointer to sensor_Data structure.
 */
void LCD_Task(void* pvParameters)
{
    sensor_Data* sensor_data = (sensor_Data*) pvParameters; // Cast parameter to sensor data
    char         buffer[32];                                // Buffer for formatted strings
    xQueueHandle lcd_reply_queue = xQueueCreate(1, sizeof(uint8_t)); // Queue for I2C replies
    vTaskDelay(pdMS_TO_TICKS(100)); // Initial delay to allow system setup

    while (1) // Infinite task loop
    {
        // Attempt to acquire OLED semaphore with 100ms timeout
        if (xSemaphoreTake(oled_sem, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            uint8_t dummy; // Dummy variable for queue receive

            // Format and display temperature (e.g., "T:23.45C")
            snprintf(buffer,
                     sizeof(buffer),
                     "T:%ld.%02ldC",
                     (long) (sensor_data->temperature / 100),
                     (long) abs(sensor_data->temperature % 100));
            lcd_display_string_via_queue(&IicInstance, buffer, 0, lcd_reply_queue);
            xQueueReceive(lcd_reply_queue, &dummy, portMAX_DELAY); // Wait for completion

            // Format and display humidity (e.g., "H:45%")
            snprintf(
                buffer, sizeof(buffer), "H:%lu%%", (unsigned long) (sensor_data->humidity / 1024));
            lcd_display_string_via_queue(&IicInstance, buffer, 2, lcd_reply_queue);
            xQueueReceive(lcd_reply_queue, &dummy, portMAX_DELAY);

            // Format and display pressure (e.g., "P:1013h")
            snprintf(
                buffer, sizeof(buffer), "P:%luh", (unsigned long) (sensor_data->pressure / 100));
            lcd_display_string_via_queue(&IicInstance, buffer, 4, lcd_reply_queue);
            xQueueReceive(lcd_reply_queue, &dummy, portMAX_DELAY);

            // Format and display luminosity (e.g., "L:200")
            snprintf(buffer, sizeof(buffer), "L:%u", sensor_data->luminosity);
            lcd_display_string_via_queue(&IicInstance, buffer, 6, lcd_reply_queue);
            xQueueReceive(lcd_reply_queue, &dummy, portMAX_DELAY);

            xSemaphoreGive(oled_sem); // Release OLED semaphore
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms
    }
}

/**
 * @brief Displays a string on the OLED using the I2C queue.
 * @details Similar to lcd_display_string, but uses the I2C task queue for
 *          thread-safe communication. Updates two pages with font data.
 * @param i2c Pointer to the I2C instance.
 * @param str Null-terminated string to display.
 * @param page Starting page (0-6).
 * @param reply_queue Queue to receive I2C completion signal.
 */
void lcd_display_string_via_queue(XIic*        i2c,
                                  const char*  str,
                                  uint8_t      page,
                                  xQueueHandle reply_queue)
{
    if (page > 7)
        return; // Validate page number

    memset(line_buffer, 0, 256); // Clear static line buffer
    uint8_t col = 0;             // Current column position

    // Convert string to font data
    for (uint32_t i = 0; str[i] != '\0' && col < 128; i++)
    {
        uint16_t font_offset = 4; // Base offset in font array
        char     c           = str[i];
        if (c >= ' ' && c <= '~')
        {
            font_offset += (c - ' ') * 16; // 16 bytes per character
        }
        for (uint8_t j = 0; j < 8; j++)
        {
            line_buffer[col + j]       = ssd1306xled_font8x16[font_offset + j];     // Upper half
            line_buffer[128 + col + j] = ssd1306xled_font8x16[font_offset + 8 + j]; // Lower half
        }
        col += 8; // Advance column by character width
    }

    i2c_request_t req   = {.reply_queue = reply_queue, .result = NULL}; // I2C request structure
    uint8_t       reply = 1;                                            // Dummy reply value

    // Upper page setup
    req.type = WRITE_LCD_CMD;
    req.cmd  = 0x21; // Set column address
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0x00; // Start column
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0x7F; // End column (127)
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0xB0 + page; // Set page address
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);

    req.type = WRITE_LCD_DATA;
    req.data = line_buffer; // Upper half data
    req.len  = 128;
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    xQueueReceive(reply_queue, &reply, portMAX_DELAY); // Wait for completion

    // Lower page setup
    req.type = WRITE_LCD_CMD;
    req.cmd  = 0x21; // Set column address
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0x00; // Start column
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0x7F; // End column (127)
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    req.cmd = 0xB0 + page + 1; // Set next page address
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);

    req.type = WRITE_LCD_DATA;
    req.data = &line_buffer[128]; // Lower half data
    req.len  = 128;
    xQueueSend(i2c_request_queue, &req, mainDONT_BLOCK);
    xQueueReceive(reply_queue, &reply, portMAX_DELAY); // Wait for completion
}

/* ===========================================================================
 * Font Data
 * ===========================================================================
 */
/* 8x16 font for SSD1306 OLED display, covering ASCII 0x20 (' ') to 0x7E ('~').
 * Each character is 16 bytes: 8 for upper half, 8 for lower half.
 */
const uint8_t ssd1306xled_font8x16[] = {
    0x00, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ' '
    0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x33, 0x30, 0x00, 0x00, 0x00, // '!'
    0x00, 0x10, 0x0C, 0x06, 0x10, 0x0C, 0x06, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // '"'
    0x40, 0xC0, 0x78, 0x40, 0xC0, 0x78, 0x40, 0x00,
    0x04, 0x3F, 0x04, 0x04, 0x3F, 0x04, 0x04, 0x00, // '#'
    0x00, 0x70, 0x88, 0xFC, 0x08, 0x30, 0x00, 0x00,
    0x00, 0x18, 0x20, 0xFF, 0x21, 0x1E, 0x00, 0x00, // '$'
    0xF0, 0x08, 0xF0, 0x00, 0xE0, 0x18, 0x00, 0x00,
    0x00, 0x21, 0x1C, 0x03, 0x1E, 0x21, 0x1E, 0x00, // '%'
    0x00, 0xF0, 0x08, 0x88, 0x70, 0x00, 0x00, 0x00,
    0x1E, 0x21, 0x23, 0x24, 0x19, 0x27, 0x21, 0x10, // '&'
    0x10, 0x16, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // '''
    0x00, 0x00, 0x00, 0xE0, 0x18, 0x04, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x07, 0x18, 0x20, 0x40, 0x00, // '('
    0x00, 0x02, 0x04, 0x18, 0xE0, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x20, 0x18, 0x07, 0x00, 0x00, 0x00, // ')'
    0x40, 0x40, 0x80, 0xF0, 0x80, 0x40, 0x40, 0x00,
    0x02, 0x02, 0x01, 0x0F, 0x01, 0x02, 0x02, 0x00, // '*'
    0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x01, 0x1F, 0x01, 0x01, 0x01, 0x00, // '+'
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0xB0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, // ','
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // '-'
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, // '.'
    0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x18, 0x04,
    0x00, 0x60, 0x18, 0x06, 0x01, 0x00, 0x00, 0x00, // '/'
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00,
    0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00, // '0'
    0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, // '1'
    0x00, 0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00,
    0x00, 0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00, // '2'
    0x00, 0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00,
    0x00, 0x18, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, // '3'
    0x00, 0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00,
    0x00, 0x07, 0x04, 0x24, 0x24, 0x3F, 0x24, 0x00, // '4'
    0x00, 0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00,
    0x00, 0x19, 0x21, 0x20, 0x20, 0x11, 0x0E, 0x00, // '5'
    0x00, 0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00,
    0x00, 0x0F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, // '6'
    0x00, 0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00,
    0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, // '7'
    0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00,
    0x00, 0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00, // '8'
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00,
    0x00, 0x00, 0x31, 0x22, 0x22, 0x11, 0x0F, 0x00, // '9'
    0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, // ':'
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, // ';'
    0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00,
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, // '<'
    0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, // '='
    0x00, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00,
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, // '>'
    0x00, 0x70, 0x48, 0x08, 0x08, 0x08, 0xF0, 0x00,
    0x00, 0x00, 0x00, 0x30, 0x36, 0x01, 0x00, 0x00, // '?'
    0xC0, 0x30, 0xC8, 0x28, 0xE8, 0x10, 0xE0, 0x00,
    0x07, 0x18, 0x27, 0x24, 0x23, 0x14, 0x0B, 0x00, // '@'
    0x00, 0x00, 0xC0, 0x38, 0xE0, 0x00, 0x00, 0x00,
    0x20, 0x3C, 0x23, 0x02, 0x02, 0x27, 0x38, 0x20, // 'A'
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00,
    0x20, 0x3F, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, // 'B'
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00,
    0x07, 0x18, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, // 'C'
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00,
    0x20, 0x3F, 0x20, 0x20, 0x20, 0x10, 0x0F, 0x00, // 'D'
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00,
    0x20, 0x3F, 0x20, 0x20, 0x23, 0x20, 0x18, 0x00, // 'E'
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00,
    0x20, 0x3F, 0x20, 0x00, 0x03, 0x00, 0x00, 0x00, // 'F'
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00,
    0x07, 0x18, 0x20, 0x20, 0x22, 0x1E, 0x02, 0x00, // 'G'
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08,
    0x20, 0x3F, 0x21, 0x01, 0x01, 0x21, 0x3F, 0x20, // 'H'
    0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, // 'I'
    0x00, 0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00,
    0xC0, 0x80, 0x80, 0x80, 0x7F, 0x00, 0x00, 0x00, // 'J'
    0x08, 0xF8, 0x88, 0xC0, 0x28, 0x18, 0x08, 0x00,
    0x20, 0x3F, 0x20, 0x01, 0x26, 0x38, 0x20, 0x00, // 'K'
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x3F, 0x20, 0x20, 0x20, 0x20, 0x30, 0x00, // 'L'
    0x08, 0xF8, 0xF8, 0x00, 0xF8, 0xF8, 0x08, 0x00,
    0x20, 0x3F, 0x00, 0x3F, 0x00, 0x3F, 0x20, 0x00, // 'M'
    0x08, 0xF8, 0x30, 0xC0, 0x00, 0x08, 0xF8, 0x08,
    0x20, 0x3F, 0x20, 0x00, 0x07, 0x18, 0x3F, 0x00, // 'N'
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00,
    0x0F, 0x10, 0x20, 0x20, 0x20, 0x10, 0x0F, 0x00, // 'O'
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x08, 0xF0, 0x00,
    0x20, 0x3F, 0x21, 0x01, 0x01, 0x01, 0x00, 0x00, // 'P'
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00,
    0x0F, 0x18, 0x24, 0x24, 0x38, 0x50, 0x4F, 0x00, // 'Q'
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00,
    0x20, 0x3F, 0x20, 0x00, 0x03, 0x0C, 0x30, 0x20, // 'R'
    0x00, 0x70, 0x88, 0x08, 0x08, 0x08, 0x38, 0x00,
    0x00, 0x38, 0x20, 0x21, 0x21, 0x22, 0x1C, 0x00, // 'S'
    0x18, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x18, 0x00,
    0x00, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x00, 0x00, // 'T'
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08,
    0x00, 0x1F, 0x20, 0x20, 0x20, 0x20, 0x1F, 0x00, // 'U'
    0x08, 0x78, 0x88, 0x00, 0x00, 0xC8, 0x38, 0x08,
    0x00, 0x00, 0x07, 0x38, 0x0E, 0x01, 0x00, 0x00, // 'V'
    0xF8, 0x08, 0x00, 0xF8, 0x00, 0x08, 0xF8, 0x00,
    0x03, 0x3C, 0x07, 0x00, 0x07, 0x3C, 0x03, 0x00, // 'W'
    0x08, 0x18, 0x68, 0x80, 0x80, 0x68, 0x18, 0x08,
    0x20, 0x30, 0x2C, 0x03, 0x03, 0x2C, 0x30, 0x20, // 'X'
    0x08, 0x38, 0xC8, 0x00, 0xC8, 0x38, 0x08, 0x00,
    0x00, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x00, 0x00, // 'Y'
    0x10, 0x08, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00,
    0x20, 0x38, 0x26, 0x21, 0x20, 0x20, 0x18, 0x00, // 'Z'
    0x00, 0x00, 0x00, 0xFE, 0x02, 0x02, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x7F, 0x40, 0x40, 0x40, 0x00, // '['
    0x00, 0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x40, 0x40, 0x7F, 0x00, 0x00, 0x00, // ']'
    0x00, 0x00, 0x04, 0x02, 0x02, 0x02, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // '^'
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, // '_'
    0x00, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // '`'
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x00, 0x19, 0x24, 0x22, 0x22, 0x22, 0x3F, 0x20, // 'a'
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x3F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, // 'b'
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x00, 0x0E, 0x11, 0x20, 0x20, 0x20, 0x11, 0x00, // 'c'
    0x00, 0x00, 0x00, 0x80, 0x80, 0x88, 0xF8, 0x00,
    0x00, 0x0E, 0x11, 0x20, 0x20, 0x10, 0x3F, 0x20, // 'd'
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x00, 0x1F, 0x22, 0x22, 0x22, 0x22, 0x13, 0x00, // 'e'
    0x00, 0x80, 0x80, 0xF0, 0x88, 0x88, 0x88, 0x18,
    0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, // 'f'
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x6B, 0x94, 0x94, 0x94, 0x93, 0x60, 0x00, // 'g'
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x20, 0x3F, 0x21, 0x00, 0x00, 0x20, 0x3F, 0x20, // 'h'
    0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, // 'i'
    0x00, 0x00, 0x00, 0x80, 0x98, 0x98, 0x00, 0x00,
    0x00, 0xC0, 0x80, 0x80, 0x80, 0x7F, 0x00, 0x00, // 'j'
    0x08, 0xF8, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00,
    0x20, 0x3F, 0x24, 0x02, 0x2D, 0x30, 0x20, 0x00, // 'k'
    0x00, 0x08, 0x08, 0xF8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, // 'l'
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
    0x20, 0x3F, 0x20, 0x00, 0x3F, 0x20, 0x00, 0x3F, // 'm'
    0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x20, 0x3F, 0x21, 0x00, 0x00, 0x20, 0x3F, 0x20, // 'n'
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
    0x00, 0x1F, 0x20, 0x20, 0x20, 0x20, 0x1F, 0x00, // 'o'
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00,
    0x80, 0xFF, 0xA1, 0x20, 0x20, 0x11, 0x0E, 0x00, // 'p'
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x0E, 0x11, 0x20, 0x20, 0xA0, 0xFF, 0x80, // 'q'
    0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00,
    0x20, 0x20, 0x3F, 0x21, 0x20, 0x00, 0x01, 0x00, // 'r'
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x33, 0x24, 0x24, 0x24, 0x24, 0x19, 0x00, // 's'
    0x00, 0x80, 0x80, 0xE0, 0x80, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1F, 0x20, 0x20, 0x00, 0x00, // 't'
    0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00,
    0x00, 0x1F, 0x20, 0x20, 0x20, 0x10, 0x3F, 0x20, // 'u'
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80,
    0x00, 0x01, 0x0E, 0x30, 0x08, 0x06, 0x01, 0x00, // 'v'
    0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x80,
    0x0F, 0x30, 0x0C, 0x03, 0x0C, 0x30, 0x0F, 0x00, // 'w'
    0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x20, 0x31, 0x2E, 0x0E, 0x31, 0x20, 0x00, // 'x'
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80,
    0x80, 0x81, 0x8E, 0x70, 0x18, 0x06, 0x01, 0x00, // 'y'
    0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x21, 0x30, 0x2C, 0x22, 0x21, 0x30, 0x00, // 'z'
    0x00, 0x00, 0x00, 0x00, 0x80, 0x7C, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x40, 0x40, // '{'
    0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, // '|'
    0x00, 0x02, 0x02, 0x7C, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00, 0x00, // '}'
    0x00, 0x06, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // '~'
};