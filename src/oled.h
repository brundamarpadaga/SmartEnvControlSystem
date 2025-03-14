/*
 * oled.h - Header file for SSD1306 OLED driver
 *
 * Purpose: Defines constants, structures, and function prototypes for controlling
 *          the ELEGOO 0.96-inch OLED display via I2C in a FreeRTOS environment.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 */

#ifndef OLED_H
#define OLED_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "xiic.h"

/* OLED I2C address (7-bit) */
#define OLED_I2C_ADDR 0x3C

/* SSD1306 command constants */
#define OLED_CMD 0x00  /* Control byte for commands */
#define OLED_DATA 0x40 /* Control byte for data */

/* OLED structure for shared data */
typedef struct
{
    XIic*             iic; /* Pointer to I2C instance */
    SemaphoreHandle_t sem; /* Semaphore for OLED access */
} OLED_t;

/* Function prototypes */
void oled_init(OLED_t* oled);
void oled_write_string(OLED_t* oled, const char* str, uint8_t page);
void OLED_Display_Task(void* p);

#endif /* OLED_H */
