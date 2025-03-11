#ifndef MAIN_H // Include guard to prevent multiple inclusions
#define MAIN_H

/*
 * main.h - Header file for FreeRTOS-based sensor and display system
 *
 * Purpose: Defines constants, structures, global variables, and function
 *          prototypes for a Microblaze-based embedded system on the Nexys A7
 *          board. Integrates sensors (TSL2561, BME280), an OLED display, and a
 *          PID controller for LED brightness, using FreeRTOS for task management.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Requires Xilinx BSP headers for hardware access, FreeRTOS headers
 *               for real-time operations, and project-specific drivers for sensors
 *               and peripherals.
 */

/*--------------------------------------------------------------------------*/
/* Standard C Library Includes                                              */
/*--------------------------------------------------------------------------*/
#include <stdint.h> // Fixed-width integer types (e.g., uint8_t, uint32_t)
#include <stdlib.h> // Standard library functions (e.g., abs() for sensor data)

/*--------------------------------------------------------------------------*/
/* Xilinx BSP and Platform Includes                                         */
/*--------------------------------------------------------------------------*/
#include "sleep.h"       // Provides usleep() for microsecond timing delays
#include "xgpio.h"       // Xilinx GPIO driver for buttons and switches
#include "xiic.h"        // Xilinx I2C driver for communication with sensors and OLED
#include "xil_printf.h"  // Lightweight printf for debug and status output
#include "xparameters.h" // Hardware-specific parameters (e.g., device IDs) from BSP
#include "xtmrctr.h"     // Xilinx timer/counter driver (unused in this project but included)

/*--------------------------------------------------------------------------*/
/* FreeRTOS Kernel Includes                                                 */
/*--------------------------------------------------------------------------*/
#include "FreeRTOS.h" // Core FreeRTOS definitions (e.g., task scheduling, configASSERT)
#include "queue.h"    // Queue management for inter-task communication
#include "semphr.h"   // Semaphore management for synchronization
#include "task.h"     // Task creation and management APIs (e.g., xTaskCreate)
#include "timers.h"   // Software timer support (unused in this project but included)

/*--------------------------------------------------------------------------*/
/* Project-Specific Includes                                                */
/*--------------------------------------------------------------------------*/
#include "bme280.h"   // BME280 sensor driver for temperature, pressure, humidity
#include "lcd.h"      // OLED (SSD1306) driver for display output
#include "nexys4IO.h" // Nexys A7 I/O peripheral driver (LEDs, buttons, 7-segment display)
#include "pidtask.h"  // PID controller definitions for LED brightness regulation
#include "platform.h" // Platform-specific configurations for Microblaze
#include "tsl2561.h"  // TSL2561 light sensor driver for lux measurements

/*--------------------------------------------------------------------------*/
/* Definitions for NEXYS4IO Peripheral                                      */
/*--------------------------------------------------------------------------*/
/**
 * @def N4IO_DEVICE_ID
 * @brief Device ID for the Nexys4IO peripheral, sourced from xparameters.h.
 */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID

/**
 * @def N4IO_BASEADDR
 * @brief Base address of the Nexys4IO peripheral in the memory map.
 */
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR

/**
 * @def N4IO_HIGHADDR
 * @brief High address of the Nexys4IO peripheral in the memory map.
 */
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

/**
 * @def I2C_BASE_ADDR
 * @brief Base address of the AXI IIC controller for I2C communication.
 */
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR

/**
 * @def I2C_DEV_ID_ADDR
 * @brief Device ID for the AXI IIC controller, sourced from xparameters.h.
 */
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

/**
 * @def configASSERT_DEFINED
 * @brief Indicates that configASSERT is defined for FreeRTOS debugging.
 */
#define configASSERT_DEFINED 1

/**
 * @def BTN_CHANNEL
 * @brief GPIO channel number for buttons (channel 1).
 */
#define BTN_CHANNEL 1

/**
 * @def SW_CHANNEL
 * @brief GPIO channel number for switches (channel 2).
 */
#define SW_CHANNEL 2

/**
 * @def mainQUEUE_LENGTH
 * @brief Length of queues used for inter-task communication (1 item).
 */
#define mainQUEUE_LENGTH ( 1 )

/**
 * @def mainDONT_BLOCK
 * @brief Non-blocking timeout for queue operations (0 ticks).
 */
#define mainDONT_BLOCK (portTickType) 0

/**
 * @def max_duty
 * @brief Maximum PWM duty cycle for RGB LED (255 = 100%).
 */
#define max_duty 255

/**
 * @def min_duty
 * @brief Minimum PWM duty cycle for RGB LED (0 = 0%).
 */
#define min_duty 0

/**
 * @def lux_mask
 * @brief Bit mask for extracting 16-bit lux or setpoint values (0xFFFF).
 */
#define lux_mask 0xFFFF

/**
 * @def UPDATE_SATURATING
 * @brief Macro to update a value with saturation limits.
 * @param val Variable to update
 * @param inc Increment/decrement amount
 * @param min_val Minimum allowed value
 * @param max_val Maximum allowed value
 * @param increase Boolean to indicate increase (true) or decrease (false)
 *
 * Description: Safely increments or decrements a value, ensuring it stays within
 *              specified min and max limits to prevent overflow/underflow.
 */
#define UPDATE_SATURATING( val, inc, min_val, max_val, increase )                                  \
    do                                                                                             \
    {                                                                                              \
        if ( increase )                                                                            \
        {                                                                                          \
            if ( ( val + inc ) < max_val )                                                         \
                val += inc;                                                                        \
            else                                                                                   \
                val = max_val;                                                                     \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            if ( val > inc )                                                                       \
                val -= inc;                                                                        \
            else                                                                                   \
                val = min_val;                                                                     \
        }                                                                                          \
    } while ( 0 )

/*--------------------------------------------------------------------------*/
/* Global Instances                                                         */
/*--------------------------------------------------------------------------*/
/**
 * @brief GPIO instance for handling buttons and switches on Nexys A7.
 *        Defined in main.c.
 */
extern XGpio xInputGPIOInstance;

/**
 * @brief I2C instance for communication with TSL2561, BME280, and OLED.
 *        Defined in main.c.
 */
extern XIic IicInstance;

/**
 * @brief Calibration data structure for BME280 sensor.
 *        Defined in main.c.
 */
extern struct bme280_calib_data calib_data;

/**
 * @brief Binary semaphore for GPIO interrupt synchronization.
 *        Defined in main.c.
 */
extern SemaphoreHandle_t binary_sem;

/**
 * @brief Semaphore for synchronizing BME280 sensor access.
 *        Defined in main.c.
 */
extern SemaphoreHandle_t bme280_sem;

/**
 * @brief Semaphore for synchronizing OLED display access.
 *        Defined in main.c.
 */
extern SemaphoreHandle_t oled_sem;

/**
 * @brief Semaphore for synchronizing I2C bus access across all peripherals.
 *        Defined in main.c.
 */
extern SemaphoreHandle_t i2c_sem;

/**
 * @brief Queue for sending button/switch data to PID task.
 *        Defined in main.c.
 */
extern xQueueHandle toPID;

/**
 * @brief Queue for sending PID setpoint and lux values to display task.
 *        Defined in main.c.
 */
extern xQueueHandle fromPID;

/*--------------------------------------------------------------------------*/
/* Structure Definitions                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @struct SensorData_t
 * @brief Structure to hold sensor data for display on the OLED.
 *
 * Description: Stores compensated sensor readings from BME280 (temperature,
 *              pressure, humidity) and TSL2561 (luminosity) with specific units.
 */
typedef struct
{
    int32_t  temperature; // Temperature in hundredths of °C (e.g., 2530 = 25.30°C)
    uint32_t pressure;    // Pressure in Pascals (Pa)
    uint32_t humidity;    // Humidity in 1024ths of % (e.g., 51200 = 50.00%)
    uint16_t luminosity;  // Luminosity in lux from TSL2561
} SensorData_t;

/**
 * @brief Global instance of SensorData_t for sharing sensor readings.
 *        Defined in main.c.
 */
extern SensorData_t sensor_data;

/*--------------------------------------------------------------------------*/
/* Function Declarations                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @brief Configure GPIO and interrupt hardware.
 *
 * Description: Initializes GPIO for buttons and switches and sets up interrupt
 *              handling. Defined in main.c.
 */
void prvSetupHardware ( void );

/**
 * @brief GPIO interrupt handler for button presses.
 * @param pvUnused Unused parameter required by interrupt handler signature
 *
 * Description: Signals the binary semaphore from ISR context and clears the
 *              interrupt. Defined in main.c.
 */
void gpio_intr ( void* pvUnused );

/**
 * @brief Initialize NX4IO and I2C hardware.
 * @return XST_SUCCESS on success, XST_FAILURE on failure
 *
 * Description: Sets up the NX4IO peripheral and I2C controller. Defined in main.c.
 */
int do_init ( void );

/**
 * @brief Task to parse button and switch inputs.
 * @param p Unused parameter
 *
 * Description: Reads GPIO inputs and sends data to PID task. Defined in main.c.
 */
void Parse_Input_Task ( void* p );

/**
 * @brief Task to manage PID control for LED brightness.
 * @param p Pointer to PID_t structure
 *
 * Description: Adjusts PID parameters and controls LED based on lux readings.
 *              Defined in main.c.
 */
void PID_Task ( void* p );

/**
 * @brief Task to update 7-segment display with PID data.
 * @param p Unused parameter
 *
 * Description: Displays setpoint and lux values on NX4IO 7-segment displays.
 *              Defined in main.c.
 */
void Display_Task ( void* p );

/**
 * @brief Initialize PID controller parameters.
 * @param pid Pointer to PID_t structure
 * @return true on success
 *
 * Description: Sets initial PID gains and limits. Defined in main.c.
 */
bool pid_init ( PID_t* pid );

/**
 * @brief Compute PID output for LED brightness control.
 * @param pid Pointer to PID_t structure
 * @param lux_value Current lux reading from TSL2561
 * @param switches Switch states to enable/disable PID terms
 * @return Normalized PID output (0 to 1)
 *
 * Description: Calculates PID output based on error and gains. Defined in main.c.
 */
float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches );

/**
 * @brief Print PID gains to console.
 * @param pid Pointer to PID_t structure
 *
 * Description: Displays Kp, Ki, and Kd values. Defined in main.c.
 */
void print_pid ( PID_t* pid );

#endif /* MAIN_H */