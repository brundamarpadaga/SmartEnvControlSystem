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

/* BME280 Macro Definitions */
#define BME280_I2C_ADDR 0x76
#define REG_CHIP_ID 0xD0
#define REG_TEMP_PRESS_CALIB 0x88
#define REG_HUMIDITY_CALIB 0xE1
#define REG_CTRL_HUM 0xF2
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_DATA 0xF7
#define BME280_CHIP_ID 0x60

/* BME280 Structure Definitions */
struct bme280_calib_data
{
    uint16_t dig_t1;
    int16_t  dig_t2;
    int16_t  dig_t3;
    uint16_t dig_p1;
    int16_t  dig_p2;
    int16_t  dig_p3;
    int16_t  dig_p4;
    int16_t  dig_p5;
    int16_t  dig_p6;
    int16_t  dig_p7;
    int16_t  dig_p8;
    int16_t  dig_p9;
    uint8_t  dig_h1;
    int16_t  dig_h2;
    uint8_t  dig_h3;
    int16_t  dig_h4;
    int16_t  dig_h5;
    int8_t   dig_h6;
    int32_t  t_fine;
};

struct bme280_uncomp_data
{
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
};

/* Global Instances */
static XGpio             xInputGPIOInstance;
XIic                     IicInstance;
struct bme280_calib_data calib_data;
SemaphoreHandle_t        binary_sem;
SemaphoreHandle_t        bme280_sem;
static xQueueHandle      toPID   = NULL;
static xQueueHandle      fromPID = NULL;

/* Function Declarations */
static void prvSetupHardware ( void );
static void gpio_intr ( void* pvUnused );
int         do_init ( void );
void        Parse_Input_Task ( void* p );
void        PID_Task ( void* p );
void        Display_Task ( void* p );
void        BME280_Task ( void* pvParameters );
bool        pid_init ( PID_t* pid );
float       pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches );
void        print_pid ( PID_t* pid );
int         bme_init ( XIic* iic );
int         bme_read_calibration_data ( XIic* iic, struct bme280_calib_data* calib );
int32_t     compensate_temperature ( const struct bme280_uncomp_data* uncomp_data,
                                     struct bme280_calib_data*        calib_data );
uint32_t    compensate_pressure ( const struct bme280_uncomp_data* uncomp_data,
                                  const struct bme280_calib_data*  calib_data );
uint32_t    compensate_humidity ( const struct bme280_uncomp_data* uncomp_data,
                                  const struct bme280_calib_data*  calib_data );

#endif /* MAIN_H */
