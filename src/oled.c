/*
 * oled.c - SSD1306 OLED driver implementation for FreeRTOS
 *
 * Purpose: Provides functions to initialize and write to the ELEGOO 0.96-inch OLED
 *          display via I2C, with semaphore protection, optimized for minimal memory use.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "oled.h"

/* Reduced font definition (24 characters: space, 0-9, T, H, P, L, F, %, a, s, c, u, x, :, .) */
static const uint8_t ssd1306xled_font8x16[] = {
    /* Space */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    /* 0 */
    0x00,
    0x00,
    0x3C,
    0x7E,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0x7E,
    0x3C,
    0x00,
    0x00,
    /* 1 */
    0x00,
    0x00,
    0x18,
    0x1C,
    0x1E,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x7E,
    0x7E,
    0x00,
    /* 2 */
    0x00,
    0x00,
    0x7E,
    0x7F,
    0x03,
    0x03,
    0x06,
    0x0C,
    0x18,
    0x30,
    0x60,
    0x60,
    0xC3,
    0xC7,
    0x7E,
    0x00,
    /* 3 */
    0x00,
    0x00,
    0x7E,
    0x7F,
    0x03,
    0x03,
    0x03,
    0x1E,
    0x1E,
    0x03,
    0x03,
    0x03,
    0x7F,
    0x7E,
    0x00,
    0x00,
    /* 4 */
    0x00,
    0x00,
    0x06,
    0x0E,
    0x1E,
    0x36,
    0x66,
    0x66,
    0xC6,
    0xFF,
    0xFF,
    0x06,
    0x06,
    0x06,
    0x00,
    0x00,
    /* 5 */
    0x00,
    0x00,
    0x7E,
    0x7F,
    0x60,
    0x60,
    0x60,
    0x7C,
    0x7E,
    0x03,
    0x03,
    0x03,
    0x7F,
    0x7E,
    0x00,
    0x00,
    /* 6 */
    0x00,
    0x00,
    0x1E,
    0x3F,
    0x70,
    0x60,
    0x60,
    0x7E,
    0x7F,
    0x63,
    0x63,
    0x63,
    0x3F,
    0x1E,
    0x00,
    0x00,
    /* 7 */
    0x00,
    0x00,
    0x7E,
    0x7F,
    0x03,
    0x03,
    0x06,
    0x06,
    0x0C,
    0x0C,
    0x18,
    0x18,
    0x30,
    0x30,
    0x00,
    0x00,
    /* 8 */
    0x00,
    0x00,
    0x3C,
    0x7E,
    0xC3,
    0xC3,
    0xC3,
    0x7E,
    0x7E,
    0xC3,
    0xC3,
    0xC3,
    0x7E,
    0x3C,
    0x00,
    0x00,
    /* 9 */
    0x00,
    0x00,
    0x3C,
    0x7E,
    0xC3,
    0xC3,
    0xC3,
    0x7F,
    0x3F,
    0x03,
    0x03,
    0x06,
    0x7C,
    0x78,
    0x00,
    0x00,
    /* T */
    0x00,
    0x00,
    0xFF,
    0xFF,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x18,
    0x00,
    0x00,
    /* H */
    0x00,
    0x00,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xFF,
    0xFF,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0xC3,
    0x00,
    0x00,
    /* P */
    0x00,
    0x00,
    0x7E,
    0x7F,
    0x63,
    0x63,
    0x63,
    0x63,
    0x7F,
    0x7E,
    0x60,
    0x60,
    0x60,
    0x60,
    0x00,
    0x00,
    /* L */
    0x00,
    0x00,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x7F,
    0x7F,
    0x00,
    0x00,
    /* F */
    0x00,
    0x00,
    0x7F,
    0x7F,
    0x60,
    0x60,
    0x60,
    0x7E,
    0x7E,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x00,
    0x00,
    /* % */
    0x00,
    0x00,
    0x63,
    0x63,
    0x66,
    0x66,
    0x0C,
    0x0C,
    0x18,
    0x18,
    0x33,
    0x33,
    0x63,
    0x63,
    0x00,
    0x00,
    /* a */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x3E,
    0x7F,
    0x03,
    0x3F,
    0x7F,
    0x63,
    0x63,
    0x7F,
    0x3E,
    0x00,
    /* s */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x3E,
    0x7F,
    0x63,
    0x70,
    0x3C,
    0x0E,
    0x63,
    0x7F,
    0x3E,
    0x00,
    /* c */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x3E,
    0x7F,
    0x63,
    0x60,
    0x60,
    0x60,
    0x63,
    0x7F,
    0x3E,
    0x00,
    /* u */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x63,
    0x63,
    0x63,
    0x63,
    0x63,
    0x63,
    0x63,
    0x7F,
    0x3E,
    0x00,
    /* x */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x63,
    0x63,
    0x36,
    0x1C,
    0x1C,
    0x36,
    0x63,
    0x63,
    0x00,
    0x00,
    /* : */
    0x00,
    0x00,
    0x00,
    0x00,
    0x18,
    0x18,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x18,
    0x18,
    0x00,
    0x00,
    /* . */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x18,
    0x18,
    0x00,
    0x00,
    0x00,
    0x00};

/* Write a command to the OLED */
static void oled_write_cmd(OLED_t* oled, uint8_t cmd)
{
    uint8_t buffer[2] = {OLED_CMD, cmd};
    XIic_Send(oled->iic->BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
}

/* Write data to the OLED */
static void oled_write_data(OLED_t* oled, uint8_t* data, uint32_t len)
{
    uint8_t buffer[9]; /* 8 bytes + control byte */
    buffer[0] = OLED_DATA;
    for (uint32_t i = 0; i < len && i < 8; i++)
    {
        buffer[i + 1] = data[i];
    }
    XIic_Send(oled->iic->BaseAddress, OLED_I2C_ADDR, buffer, len + 1, XIIC_STOP);
}

/*
 * oled_init - Initialize the OLED display
 */
void oled_init(OLED_t* oled)
{
    oled_write_cmd(oled, 0xAE); /* Display OFF */
    oled_write_cmd(oled, 0xD5);
    oled_write_cmd(oled, 0x80); /* Clock */
    oled_write_cmd(oled, 0xA8);
    oled_write_cmd(oled, 0x3F); /* MUX */
    oled_write_cmd(oled, 0xD3);
    oled_write_cmd(oled, 0x00); /* Offset */
    oled_write_cmd(oled, 0x40); /* Start line */
    oled_write_cmd(oled, 0x8D);
    oled_write_cmd(oled, 0x14); /* Charge pump */
    oled_write_cmd(oled, 0x20);
    oled_write_cmd(oled, 0x00); /* Horizontal mode */
    oled_write_cmd(oled, 0xA0); /* Segment remap */
    oled_write_cmd(oled, 0xC0); /* COM scan direction */
    oled_write_cmd(oled, 0xDA);
    oled_write_cmd(oled, 0x12); /* COM pins */
    oled_write_cmd(oled, 0x81);
    oled_write_cmd(oled, 0xCF); /* Contrast */
    oled_write_cmd(oled, 0xD9);
    oled_write_cmd(oled, 0xF1); /* Pre-charge */
    oled_write_cmd(oled, 0xDB);
    oled_write_cmd(oled, 0x40); /* VCOMH */
    oled_write_cmd(oled, 0xA4); /* Resume RAM */
    oled_write_cmd(oled, 0xA6); /* Normal display */
    oled_write_cmd(oled, 0xAF); /* Display ON */
}

/*
 * oled_write_string - Write a string to the OLED at specified page
 */
void oled_write_string(OLED_t* oled, const char* str, uint8_t page)
{
    if (xSemaphoreTake(oled->sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        /* Clear page (2 pages tall) */
        uint8_t clear_buffer[8] = {0};
        for (uint8_t p = page; p < page + 2; p++)
        {
            oled_write_cmd(oled, 0xB0 + p);
            oled_write_cmd(oled, 0x00);
            oled_write_cmd(oled, 0x10);
            for (uint8_t i = 0; i < 16; i++)
            { /* 128 / 8 = 16 chunks */
                oled_write_data(oled, clear_buffer, 8);
            }
        }

        /* Write string */
        uint8_t col = 0;
        for (uint32_t i = 0; str[i] != '\0' && col < 128; i++)
        {
            uint16_t font_offset = 0;
            switch (str[i])
            {
                case ' ':
                    font_offset = 0 * 16;
                    break;
                case '0':
                    font_offset = 1 * 16;
                    break;
                case '1':
                    font_offset = 2 * 16;
                    break;
                case '2':
                    font_offset = 3 * 16;
                    break;
                case '3':
                    font_offset = 4 * 16;
                    break;
                case '4':
                    font_offset = 5 * 16;
                    break;
                case '5':
                    font_offset = 6 * 16;
                    break;
                case '6':
                    font_offset = 7 * 16;
                    break;
                case '7':
                    font_offset = 8 * 16;
                    break;
                case '8':
                    font_offset = 9 * 16;
                    break;
                case '9':
                    font_offset = 10 * 16;
                    break;
                case 'T':
                    font_offset = 11 * 16;
                    break;
                case 'H':
                    font_offset = 12 * 16;
                    break;
                case 'P':
                    font_offset = 13 * 16;
                    break;
                case 'L':
                    font_offset = 14 * 16;
                    break;
                case 'F':
                    font_offset = 15 * 16;
                    break;
                case '%':
                    font_offset = 16 * 16;
                    break;
                case 'a':
                    font_offset = 17 * 16;
                    break;
                case 's':
                    font_offset = 18 * 16;
                    break;
                case 'c':
                    font_offset = 19 * 16;
                    break;
                case 'u':
                    font_offset = 20 * 16;
                    break;
                case 'x':
                    font_offset = 21 * 16;
                    break;
                case ':':
                    font_offset = 22 * 16;
                    break;
                case '.':
                    font_offset = 23 * 16;
                    break;
                default:
                    font_offset = 0 * 16;
                    break;
            }

            oled_write_cmd(oled, 0xB0 + page);
            oled_write_cmd(oled, 0x00 + (col & 0x0F));
            oled_write_cmd(oled, 0x10 + (col >> 4));
            oled_write_data(oled, (uint8_t*) &ssd1306xled_font8x16[font_offset], 8);

            oled_write_cmd(oled, 0xB0 + page + 1);
            oled_write_cmd(oled, 0x00 + (col & 0x0F));
            oled_write_cmd(oled, 0x10 + (col >> 4));
            oled_write_data(oled, (uint8_t*) &ssd1306xled_font8x16[font_offset + 8], 8);

            col += 8;
        }
        xSemaphoreGive(oled->sem);
    }
}
