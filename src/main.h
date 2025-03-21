/*
 * ===========================================================================
 * File:    main.h
 * Summary: Main Header File for Environmental Control System
 * ===========================================================================
 * Purpose: This header consolidates definitions, includes, and declarations
 *          for the FreeRTOS-based environmental control system on the Nexys A7
 *          board. It provides macros, structures, and global variables for
 *          interfacing with sensors (BME280, TSL2561), OLED display, GPIO,
 *          and PID control tasks.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 * Author:  [Your Name/Group Name, if applicable]
 *
 * Dependencies:
 *   - Standard C: stdint.h, stdlib.h
 *   - Xilinx BSP: sleep.h, xgpio.h, xiic.h, xil_printf.h, xparameters.h, xtmrctr.h
 *   - FreeRTOS: FreeRTOS.h, queue.h, semphr.h, task.h, timers.h
 *   - Project: bme280.h, lcd.h, nexys4IO.h, pidtask.h, platform.h, tsl2561.h
 *
 * Notes:
 *   - Designed for the Nexys A7 board with Microblaze processor.
 *   - Includes custom I2C request types and utility macros for system integration.
 *   - All global instances are defined in main.c and externed here.
 */

/* ===========================================================================
 * Header Guard
 * ===========================================================================
 */
#ifndef MAIN_H
#define MAIN_H

/* ===========================================================================
 * Standard C Library Includes
 * ===========================================================================
 */
#include <stdint.h> // Fixed-width integer types (e.g., uint16_t, int32_t)
#include <stdlib.h> // Standard library functions (e.g., abs)

/* ===========================================================================
 * Xilinx BSP and Platform Includes
 * ===========================================================================
 */
#include "sleep.h"       // Sleep functions (e.g., usleep)
#include "xgpio.h"       // GPIO driver for buttons and switches
#include "xiic.h"        // Xilinx I2C driver for sensor/display communication
#include "xil_printf.h"  // Lightweight printf for logging
#include "xparameters.h" // Hardware configuration parameters
#include "xtmrctr.h"     // Timer counter driver (if used)

/* ===========================================================================
 * FreeRTOS Kernel Includes
 * ===========================================================================
 */
#include "FreeRTOS.h" // Core FreeRTOS definitions and macros
#include "queue.h"    // Queue management for task communication
#include "semphr.h"   // Semaphore support for synchronization
#include "task.h"     // Task management functions
#include "timers.h"   // Software timer support

/* ===========================================================================
 * Project-Specific Includes
 * ===========================================================================
 */
#include "bme280.h"   // BME280 sensor driver definitions
#include "lcd.h"      // OLED display driver definitions
#include "nexys4IO.h" // Nexys4IO peripheral driver
#include "pidtask.h"  // PID control task definitions
#include "platform.h" // Platform-specific configurations
#include "tsl2561.h"  // TSL2561 light sensor driver definitions

/* ===========================================================================
 * Peripheral Definitions
 * ===========================================================================
 */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID       // Nexys4IO device ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR // Base address for Nexys4IO
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR // High address for Nexys4IO
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR          // I2C base address
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID       // I2C device ID

#define BTN_CHANNEL 1 // GPIO channel for buttons (channel 1)
#define SW_CHANNEL 2  // GPIO channel for switches (channel 2)

/* ===========================================================================
 * Configuration Constants
 * ===========================================================================
 */
#define mainQUEUE_LENGTH 1                     // Queue length for task communication
#define mainDONT_BLOCK ((TickType_t) 0)        // Non-blocking queue/tick operation
#define QUEUE_TIMEOUT_TICKS pdMS_TO_TICKS(10)  // 10ms timeout for queue operations
#define DEBOUNCE_DELAY_TICKS pdMS_TO_TICKS(20) // 20ms debounce period for inputs

#define max_duty 254    // Maximum PWM duty cycle (8-bit resolution)
#define min_duty 0      // Minimum PWM duty cycle
#define lux_mask 0xFFFF // 16-bit mask for lux values

/* ===========================================================================
 * Logging Levels
 * ===========================================================================
 */
#define LOG_LEVEL_ERROR 1          // Error logging level (highest priority)
#define LOG_LEVEL_WARNING 2        // Warning logging level
#define LOG_LEVEL_INFO 3           // Info logging level (lowest priority)
#define DEBUG_LEVEL LOG_LEVEL_INFO // Default debug level

/* ===========================================================================
 * Bit Masks
 * ===========================================================================
 */
#define BUTTON_MASK 0x1F // Mask for 5 button bits (BTN0-BTN4)
#define SWITCH_MASK 0xFF // Mask for 8 switch bits (SW0-SW7)

/* ===========================================================================
 * Utility Macro
 * ===========================================================================
 */
/**
 * @brief Updates a value with saturation limits.
 * @details Adds or subtracts an increment, clamping to min/max bounds.
 * @param val Variable to update
 * @param inc Increment/decrement amount
 * @param min_val Minimum allowed value
 * @param max_val Maximum allowed value
 * @param increase Boolean: true to increase, false to decrease
 */
#define UPDATE_SATURATING(val, inc, min_val, max_val, increase)                                    \
    do                                                                                             \
    {                                                                                              \
        if (increase)                                                                              \
        {                                                                                          \
            if ((val + inc) < max_val)                                                             \
                val += inc;                                                                        \
            else                                                                                   \
                val = max_val;                                                                     \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            if (val > inc)                                                                         \
                val -= inc;                                                                        \
            else                                                                                   \
                val = min_val;                                                                     \
        }                                                                                          \
    } while (0)

/* ===========================================================================
 * I2C Request Types and Structure
 * ===========================================================================
 */
/**
 * @brief Enum for I2C request types.
 * @details Defines operations for TSL2561, BME280, and LCD interactions.
 */
typedef enum
{
    READ_TSL2561_CH0,  // Read TSL2561 Channel 0 (visible + IR)
    READ_TSL2561_CH1,  // Read TSL2561 Channel 1 (IR only)
    READ_BME280_TEMP,  // Read BME280 temperature
    READ_BME280_HUM,   // Read BME280 humidity
    READ_BME280_PRESS, // Read BME280 pressure
    WRITE_LCD_CMD,     // Write command to OLED display
    WRITE_LCD_DATA     // Write data to OLED display
} i2c_request_type_t;

/**
 * @brief Structure for I2C requests.
 * @details Encapsulates details for I2C operations handled by the I2C task.
 */
typedef struct
{
    i2c_request_type_t type;        // Type of I2C operation
    float*             result;      // Pointer to store read result (NULL for writes)
    uint8_t*           data;        // Data buffer for LCD writes (NULL otherwise)
    uint32_t           len;         // Length of data for LCD writes
    uint8_t            cmd;         // Command byte for LCD writes
    QueueHandle_t      reply_queue; // Queue to receive operation reply (NULL for writes)
} i2c_request_t;

/* ===========================================================================
 * Global Instances
 * ===========================================================================
 */
extern SemaphoreHandle_t        binary_sem;         // Semaphore for GPIO interrupt sync
extern SemaphoreHandle_t        bme280_sem;         // Semaphore for BME280 access
extern SemaphoreHandle_t        oled_sem;           // Semaphore for OLED access
extern SemaphoreHandle_t        i2c_sem;            // Semaphore for I2C bus access
extern XGpio                    xInputGPIOInstance; // GPIO instance for buttons/switches
extern QueueHandle_t            toPID;              // Queue for sending data to PID task
extern QueueHandle_t            fromPID;            // Queue for receiving data from PID task
extern QueueHandle_t            i2c_request_queue;  // Queue for I2C transaction requests
extern XIic                     IicInstance;        // Global I2C instance
extern struct bme280_calib_data calib_data;         // BME280 calibration data

/* ===========================================================================
 * Function Declarations
 * ===========================================================================
 */

/**
 * @brief Configures GPIO and interrupt system hardware.
 */
void prvSetupHardware(void);

/**
 * @brief GPIO interrupt service routine.
 * @param pvUnused Unused parameter for ISR signature.
 */
void gpio_intr(void* pvUnused);

/**
 * @brief Initializes system peripherals (NX4IO, I2C, RGB LEDs).
 * @return int XST_SUCCESS on success, XST_FAILURE on failure.
 */
int do_init(void);

/**
 * @brief Task to process button and switch inputs.
 * @param p Unused parameter for task signature.
 */
void Parse_Input_Task(void* p);

/**
 * @brief Task to execute PID control and update actuators.
 * @param p Pointer to sensor_Data structure.
 */
void PID_Task(void* p);

/**
 * @brief Task to update 7-segment display with PID data.
 * @param p Unused parameter for task signature.
 */
void Display_Task(void* p);

/**
 * @brief Adjusts actuator duty cycle with PID correction.
 * @param enviro Current duty cycle.
 * @param pidOut PID correction percentage.
 * @param fanCtrl True if controlling fan (inverts correction).
 * @return uint8_t Updated duty cycle.
 */
uint8_t correctedSignal(uint8_t enviro, float pidOut, bool fanCtrl);

/**
 * @brief Helper to update 7-segment display with PID data.
 * @param pid Pointer to PID structure.
 * @param btns Button states.
 * @param sensorVal Current sensor value.
 * @param incr Increment scaler.
 */
void displayHelper(PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr);

/**
 * @brief Cleans up system resources on shutdown or failure.
 */
void cleanup_system(void);

/**
 * @brief Task to handle I2C transactions via queue.
 * @param p Unused parameter for task signature.
 */
void I2C_Task(void* p);

#endif /* MAIN_H */