#ifndef TSL2561_H // Include guard to prevent multiple inclusions
#define TSL2561_H

/*
 * tsl2561.h - Header file for TSL2561 luminosity sensor driver
 *
 * Purpose: Defines constants, types, and function prototypes for initializing
 *          and reading the TSL2561 ambient light sensor over I2C in polling mode.
 *          Provides support for luminance measurement in a FreeRTOS-based system
 *          on the Nexys A7 with a Microblaze processor, targeting broadband and
 *          infrared light readings.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Relies on Xilinx BSP headers for I2C and timing, FreeRTOS headers
 *               for real-time functionality, and standard C types for portability.
 */

/*--------------------------------------------------------------------------*/
/* FreeRTOS Includes - Core FreeRTOS headers for real-time functionality    */
/*--------------------------------------------------------------------------*/
#include "FreeRTOS.h" // Main FreeRTOS kernel definitions (e.g., TickType_t, pdMS_TO_TICKS)
#include "semphr.h"   // Semaphore APIs for synchronization (e.g., SemaphoreHandle_t)
#include "task.h"     // Task management APIs (e.g., vTaskDelay for timing control)

/*--------------------------------------------------------------------------*/
/* BSP Includes - Xilinx BSP headers for hardware interaction              */
/*--------------------------------------------------------------------------*/
#include "sleep.h"      // Provides usleep() for microsecond delays during sensor operations
#include "xiic.h"       // Xilinx I2C driver API for AXI IIC controller communication with TSL2561
#include "xil_printf.h" // Lightweight printf implementation for debug and error output

/*--------------------------------------------------------------------------*/
/* Standard Includes - General-purpose C library types                     */
/*--------------------------------------------------------------------------*/
#include <stdint.h> // Fixed-width integer types (e.g., uint16_t for sensor data)

/*--------------------------------------------------------------------------*/
/* TSL2561 Device Address                                                  */
/*--------------------------------------------------------------------------*/
/**
 * @def TSL2561_ADDR
 * @brief Default 7-bit I2C slave address of the TSL2561 sensor (0x39).
 *        Set when the ADDR pin is floating; alternatives are 0x29 (ADDR low) or
 *        0x49 (ADDR high).
 */
#define TSL2561_ADDR 0x39

/*--------------------------------------------------------------------------*/
/* Register Addresses (Base Values) - Raw offsets without command bit      */
/*--------------------------------------------------------------------------*/
/**
 * @def CONTROL_REG
 * @brief Base address of the Control register (0x00).
 *        Used to power the sensor on/off.
 */
#define CONTROL_REG 0x00

/**
 * @def TIMING_REG
 * @brief Base address of the Timing register (0x01).
 *        Configures integration time and gain settings.
 */
#define TIMING_REG 0x01

/**
 * @def ID_REG
 * @brief Base address of the ID register (0x0A).
 *        Contains device identification (not used in this implementation).
 */
#define ID_REG 0x0A

/**
 * @def DATA0LOW_REG
 * @brief Base address of Channel 0 low byte register (0x0C).
 *        Stores the lower 8 bits of broadband (Visible + IR) light data.
 */
#define DATA0LOW_REG 0x0C

/**
 * @def DATA0HIGH_REG
 * @brief Base address of Channel 0 high byte register (0x0D).
 *        Stores the upper 8 bits of broadband (Visible + IR) light data.
 */
#define DATA0HIGH_REG 0x0D

/**
 * @def DATA1LOW_REG
 * @brief Base address of Channel 1 low byte register (0x0E).
 *        Stores the lower 8 bits of infrared-only light data.
 */
#define DATA1LOW_REG 0x0E

/**
 * @def DATA1HIGH_REG
 * @brief Base address of Channel 1 high byte register (0x0F).
 *        Stores the upper 8 bits of infrared-only light data.
 */
#define DATA1HIGH_REG 0x0F

/*--------------------------------------------------------------------------*/
/* Command Bytes - Register addresses with command bit (0x80) set          */
/*--------------------------------------------------------------------------*/
/**
 * @def TSL2561_CMD_CONTROL
 * @brief Command byte for the Control register (0x80 = 0x00 | 0x80).
 *        Indicates a command transaction to the control register.
 */
#define TSL2561_CMD_CONTROL 0x80

/**
 * @def TSL2561_CMD_TIMING
 * @brief Command byte for the Timing register (0x81 = 0x01 | 0x80).
 *        Indicates a command transaction to set integration time and gain.
 */
#define TSL2561_CMD_TIMING 0x81

/**
 * @def TSL2561_CMD_INTERRUPT
 * @brief Command byte for the Interrupt register (0x86 = 0x06 | 0x80).
 *        Unused in this polling-based implementation.
 */
#define TSL2561_CMD_INTERRUPT 0x86

/**
 * @def TSL2561_CMD_DATA0
 * @brief Command byte for Channel 0 low byte register (0x8C = 0x0C | 0x80).
 *        Indicates a data read from Channel 0 (broadband).
 */
#define TSL2561_CMD_DATA0 0x8C

/**
 * @def TSL2561_CMD_DATA1
 * @brief Command byte for Channel 1 low byte register (0x8E = 0x0E | 0x80).
 *        Indicates a data read from Channel 1 (infrared).
 */
#define TSL2561_CMD_DATA1 0x8E

/*--------------------------------------------------------------------------*/
/* Register Values - Predefined configuration values                       */
/*--------------------------------------------------------------------------*/
/**
 * @def TSL2561_POWER_ON
 * @brief Value to power on the TSL2561 (0x03).
 *        Sets the sensor to active mode in the Control register.
 */
#define TSL2561_POWER_ON 0x03

/**
 * @def TSL2561_POWER_OFF
 * @brief Value to power off the TSL2561 (0x00).
 *        Sets the sensor to inactive mode in the Control register.
 */
#define TSL2561_POWER_OFF 0x00

/**
 * @def TSL2561_TIMING_402MS_16X
 * @brief Timing register value for 402ms integration time and 16x gain (0x02).
 *        Configures the sensor for high sensitivity and longer integration.
 */
#define TSL2561_TIMING_402MS_16X 0x02

/**
 * @def TSL2561_INT_DISABLE
 * @brief Value to disable interrupts in the Interrupt register (0x00).
 *        Unused in this polling-based implementation.
 */
#define TSL2561_INT_DISABLE 0x00

/*--------------------------------------------------------------------------*/
/* Timeout Definition                                                      */
/*--------------------------------------------------------------------------*/
/**
 * @def TIMEOUT_COUNTER
 * @brief Maximum number of cycles to wait for I2C bus to become idle (1,000,000).
 *        Adjustable based on system clock speed to prevent infinite loops.
 */
#define TIMEOUT_COUNTER 1000000

/*--------------------------------------------------------------------------*/
/* Enum Definitions                                                        */
/*--------------------------------------------------------------------------*/
/**
 * @enum tsl2561_channel_t
 * @brief Enumeration for TSL2561 sensor channels.
 *
 * Description: Defines the two available channels for readability and type safety.
 */
typedef enum
{
    TSL2561_CHANNEL_0 = 0, // Channel 0: Broadband (Visible light + Infrared)
    TSL2561_CHANNEL_1 = 1  // Channel 1: Infrared only
} tsl2561_channel_t;

/*--------------------------------------------------------------------------*/
/* Function Prototypes                                                     */
/*--------------------------------------------------------------------------*/
/**
 * @brief Initialize the TSL2561 sensor over I2C.
 * @param i2c Pointer to the initialized XIic instance
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 *
 * Description: Powers on the TSL2561, resets the I2C peripheral, and configures
 *              the sensor with 402ms integration time and 16x gain in polling mode.
 *              Ensures proper setup for subsequent readings.
 */
int tsl2561_init ( XIic* i2c );

/**
 * @brief Read a channel (CH0 or CH1) from the TSL2561 sensor.
 * @param i2c Pointer to the initialized XIic instance
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1)
 * @return Raw 16-bit channel value as a float on success, 0 on failure
 *
 * Description: Sends the appropriate register address and reads two bytes (low
 *              and high) to form a 16-bit ADC value representing light intensity.
 *              Returns 0 if I2C communication fails.
 */
float tsl2561_readChannel ( XIic* i2c, tsl2561_channel_t channel );

/**
 * @brief Calculate lux value from CH0 and CH1 readings (placeholder).
 * @param ch0 Channel 0 value (Visible + IR, broadband)
 * @param ch1 Channel 1 value (IR only)
 * @return Lux value as a float
 *
 * Description: Intended to convert raw ADC values from both channels into a lux
 *              measurement using the TSL2561 datasheet formula. Not implemented
 *              in the current driver but included for future expansion.
 */
float tsl2561_calculateLux ( uint16_t ch0, uint16_t ch1 );

#endif /* TSL2561_H */