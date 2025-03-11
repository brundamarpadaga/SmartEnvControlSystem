#ifndef LCD_H
#define LCD_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "xiic.h"
#include "xil_printf.h"

#define OLED_I2C_ADDR 0x3C
#define OLED_CMD  0x00
#define OLED_DATA 0x40

extern XIic IicInstance;
extern SemaphoreHandle_t oled_sem;
extern SemaphoreHandle_t i2c_sem;
extern const uint8_t ssd1306xled_font8x16[];

int lcd_init(XIic* i2c);
int lcd_display_string(XIic* i2c, const char* str, uint8_t page);
void LCD_Task(void* pvParameters);

#endif /* LCD_H */
