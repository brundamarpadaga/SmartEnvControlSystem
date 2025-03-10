#ifndef MAIN_H
#define MAIN_H

/*
 * main.h - Header file for TSL2561 and BME280-based PID control system with OLED display
 *
 * Purpose: Defines constants, global variables, and function prototypes for the
 *          FreeRTOS-based environmental control system. This includes hardware interfaces
 *          (GPIO, I2C, NX4IO), sensor drivers (TSL2561, BME280), PID control, and OLED display
 *          functionality for dynamic sensor data visualization. BME280-specific structures
 *          and functions are defined in bme280.h to avoid redefinition.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Final Project - Environmental Control System
 */

/* Standard C library includes */
#include <stdint.h>
#include <stdlib.h>

/* Xilinx BSP and platform includes */
#include "sleep.h"
#include "xgpio.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xtmrctr.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* Project-specific includes */
#include "bme280.h"
#include "nexys4IO.h"
#include "oled.h"
#include "pidtask.h"
#include "platform.h"
#include "tsl2561.h"

/* Definitions for NEXYS4IO Peripheral */
#define N4IO_DEVICE_ID  XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR   XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR   XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR   XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

/* GPIO channel definitions */
#define BTN_CHANNEL 1
#define SW_CHANNEL  2

/* Queue and timing definitions */
#define mainQUEUE_LENGTH 1
#define mainDONT_BLOCK (TickType_t)0

/* PID control definitions */
#define max_duty 255
#define min_duty 0
#define lux_mask 0xFFFF

#define UPDATE_SATURATING(val, inc, min_val, max_val, increase) \
    do {                                                        \
        if (increase) {                                         \
            if ((val + inc) < max_val)                          \
                val += inc;                                     \
            else                                                \
                val = max_val;                                  \
        } else {                                                \
            if (val > inc)                                      \
                val -= inc;                                     \
            else                                                \
                val = min_val;                                  \
        }                                                       \
    } while (0)

/* Global Instances */
extern XGpio xInputGPIOInstance;
extern XIic IicInstance;
extern SemaphoreHandle_t binary_sem;
extern xQueueHandle toPID;
extern xQueueHandle fromPID;

/* Function Declarations */
void prvSetupHardware(void); /* Removed static */
static void gpio_intr(void* pvUnused);
int do_init(void);
void Parse_Input_Task(void* p);
void PID_Task(void* p);
void OLED_Display_Task(void* p);
void BME280_Task(void* pvParameters);
bool pid_init(PID_t* pid);
float pid_funct(PID_t* pid, uint16_t lux_value, uint8_t switches);
void print_pid(PID_t* pid);

#endif /* MAIN_H */
