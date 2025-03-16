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
#include "nexys4IO.h"
#include "pidtask.h"
#include "platform.h"
#include "tsl2561.h"
#include "bme280.h"
#include "lcd.h"

/* Definitions for NEXYS4IO Peripheral */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

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
extern SemaphoreHandle_t binary_sem;
extern XGpio             xInputGPIOInstance;
extern xQueueHandle toPID;
extern xQueueHandle fromPID;
extern XIic IicInstance;
extern struct bme280_calib_data calib_data;

/* Function Declarations */
extern void prvSetupHardware ( void );
extern void gpio_intr ( void* pvUnused );
extern int         do_init ( void );
extern void        Parse_Input_Task ( void* p );
extern void        PID_Task ( void* p );
extern void        Display_Task ( void* p );
extern uint8_t correctedSignal (uint8_t enviro, float pidOut, bool fanCrtl);
extern void displayHelper (PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr);
#endif /* MAIN_H */
