#ifndef MAIN_H
#define MAIN_H

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
#include "lcd.h"
#include "nexys4IO.h"
#include "pidtask.h"
#include "platform.h"
#include "tsl2561.h"

/* Definitions for NEXYS4IO Peripheral */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

#define configASSERT_DEFINED 1

#define BTN_CHANNEL 1
#define SW_CHANNEL 2

#define mainQUEUE_LENGTH ( 1 )
#define mainDONT_BLOCK (portTickType) 0

#define max_duty 255
#define min_duty 0
#define lux_mask 0xFFFF

#define UPDATE_SATURATING( val, inc, min_val, max_val, increase ) \
    do                                                            \
    {                                                             \
        if ( increase )                                           \
        {                                                         \
            if ( ( val + inc ) < max_val )                        \
                val += inc;                                       \
            else                                                  \
                val = max_val;                                    \
        }                                                         \
        else                                                      \
        {                                                         \
            if ( val > inc )                                      \
                val -= inc;                                       \
            else                                                  \
                val = min_val;                                    \
        }                                                         \
    } while ( 0 )

/* Global Instances */
extern XGpio                    xInputGPIOInstance;
extern XIic                     IicInstance;
extern struct bme280_calib_data calib_data;
extern SemaphoreHandle_t        binary_sem;
extern SemaphoreHandle_t        bme280_sem;
extern SemaphoreHandle_t        oled_sem;
extern SemaphoreHandle_t        i2c_sem;  // Global I2C semaphore
extern xQueueHandle             toPID;
extern xQueueHandle             fromPID;

/* Structure to hold sensor data for LCD */
typedef struct
{
    int32_t  temperature; /* In hundredths of �C */
    uint32_t pressure;    /* In Pa */
    uint32_t humidity;    /* In 1024ths of % */
    uint16_t luminosity;  /* In lux */
} SensorData_t;

extern SensorData_t sensor_data;

/* Function Declarations */
void  prvSetupHardware ( void );     // Remove static, defined in main.c
void  gpio_intr ( void* pvUnused );  // Remove static, defined in main.c
int   do_init ( void );
void  Parse_Input_Task ( void* p );
void  PID_Task ( void* p );
void  Display_Task ( void* p );
bool  pid_init ( PID_t* pid );
float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches );
void  print_pid ( PID_t* pid );

#endif /* MAIN_H */
