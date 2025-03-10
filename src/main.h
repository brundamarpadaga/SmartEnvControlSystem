

#ifndef MAIN_H
#define MAIN_H

/* Standard C library includes. */
#include <stdlib.h>

/* Xilinx BSP and platform includes. */
#include "sleep.h"
#include "xgpio.h"
#include "xiic.h"  // Xilinx I2C driver API for AXI IIC controller
#include "xil_printf.h"
#include "xparameters.h"
#include "xtmrctr.h"
// #include "xintc.h" // Xilinx interrupt controller API for interrupt
// management

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "xstatus.h"

/* Project-specific includes. */
#include "i2c.h"  // I2C driver interface for AXI IIC controller
#include "nexys4IO.h"
#include "pidtask.h"
#include "platform.h"
#include "tsl2561.h"

/*Definitions for NEXYS4IO Peripheral*/
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

#define BTN_CHANNEL 1
#define SW_CHANNEL 2

#define mainQUEUE_LENGTH (1)

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK (portTickType) 0

/********** Duty Cycle Related Constants **********/
#define max_duty 255  // max possible duty cycle for RGB1 Blue
#define min_duty 0    // min possible duty cycle for RGB1 Blue

#define lux_mask 0xFFFF  // mask used for combining setpoint and lux values into one double

// macro for repeating code that checks for upper/lower saturation of setpoint
// and gains
#define UPDATE_SATURATING(val, inc, min_val, max_val, increase) \
    do                                                          \
    {                                                           \
        if (increase)                                           \
        {                                                       \
            if ((val + inc) < max_val)                          \
                val += inc;                                     \
            else                                                \
                val = max_val;                                  \
        }                                                       \
        else                                                    \
        {                                                       \
            if (val > inc)                                      \
                val -= inc;                                     \
            else                                                \
                val = min_val;                                  \
        }                                                       \
    } while (0)

// Create Instances
static XGpio xInputGPIOInstance;

// Declare a Semaphore
xSemaphoreHandle binary_sem;

/* Interrupt-related globals (to be defined in tsl2561.c) */
XIic IicInstance;  // I2C instance
// XIntc Intc; // Shared interrupt controller instance, defined in main.c

/* The queue used to transfer buttons/switches from the input task to PID task.
 */
static xQueueHandle toPID = NULL;

/* The queue used to transfer setpoint and lux from PID to display task. */
static xQueueHandle fromPID = NULL;

// Function Declarations
static void prvSetupHardware(void);
static void gpio_intr(void* pvUnused);

// A task which takes the Interrupt Semaphore and sends the btn/sw states to PID
// Task
void Parse_Input_Task(void* p);

int do_init(void);

#endif /* MAIN_H */
