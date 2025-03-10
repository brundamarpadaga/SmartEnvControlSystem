
#ifndef TSL2561_H
#define TSL2561_H

/*
 * tsl2561.h - Header file for TSL2561 luminosity sensor driver
 *
 * Purpose: Defines constants, types, and function prototypes for initializing
 *          and reading the TSL2561 sensor over I2C in polling mode. Provides
 *          support for luminance measurement in a FreeRTOS-based system on
 *          the Nexys A7 with Microblaze.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors:
 */

/* FreeRTOS includes - Core FreeRTOS headers for real-time functionality */
#include "FreeRTOS.h"  // Main FreeRTOS kernel definitions and types (e.g., TickType_t)
#include "semphr.h"  // Semaphore APIs for synchronization (unused here but included for potential future use)
#include "task.h"  // Task management APIs (e.g., vTaskDelay)

/* BSP includes - Xilinx BSP headers for hardware interaction */
#include "sleep.h"       // Provides usleep() for timing delays in sensor operations
#include "xiic.h"        // Xilinx I2C driver API for AXI IIC controller communication
#include "xil_printf.h"  // Xilinx printf implementation for debug output

/* Standard includes - General-purpose C library types */
#include <stdint.h>  // Provides fixed-width integer types (e.g., uint16_t)

/* TSL2561 device address - I2C slave address for the sensor */
#define TSL2561_ADDR 0x39  // Default I2C address (ADDR pin floating, 7-bit address)

/* Register addresses (base values) - Raw register offsets without command bit
 */
#define CONTROL_REG 0x00    // Control register for power and operation control
#define TIMING_REG 0x01     // Timing register for integration time and gain settings
#define ID_REG 0x0A         // ID register for device identification
#define DATA0LOW_REG 0x0C   // Channel 0 low byte (Visible + IR)
#define DATA0HIGH_REG 0x0D  // Channel 0 high byte (Visible + IR)
#define DATA1LOW_REG 0x0E   // Channel 1 low byte (IR only)
#define DATA1HIGH_REG 0x0F  // Channel 1 high byte (IR only)

/* Command bytes - Register addresses with command bit (0x80) set for I2C
 * transactions */
#define TSL2561_CMD_CONTROL 0x80  // Command byte for Control register (0x00 | 0x80)
#define TSL2561_CMD_TIMING 0x81   // Command byte for Timing register (0x01 | 0x80)
#define TSL2561_CMD_INTERRUPT \
    0x86                        // Command byte for Interrupt register (0x06 | 0x80, unused here)
#define TSL2561_CMD_DATA0 0x8C  // Command byte for Data0 low byte (0x0C | 0x80)
#define TSL2561_CMD_DATA1 0x8E  // Command byte for Data1 low byte (0x0E | 0x80)

/* Register values - Predefined values for configuring TSL2561 registers */
#define TSL2561_POWER_ON 0x03          // Power-on value for Control register (active mode)
#define TSL2561_POWER_OFF 0x00         // Power-off value for Control register (inactive mode)
#define TSL2561_TIMING_402MS_16X 0x02  // Timing register value: 402ms integration time, 16x gain
#define TSL2561_INT_DISABLE 0x00       // Interrupt register value: Disable interrupts (unused here)

/* Timeout for I2C operations - Maximum cycles to wait for bus idle */
#define TIMEOUT_COUNTER \
    1000000  // Timeout counter for I2C busy checks, adjustable based on system
             // clock

/* Enum for TSL2561 channels - Defines the two sensor channels for readability
 */
typedef enum
{
    TSL2561_CHANNEL_0 = 0,  // Channel 0: Visible light + Infrared (broadband)
    TSL2561_CHANNEL_1 = 1   // Channel 1: Infrared only
} tsl2561_channel_t;

/* Function prototypes - Declarations for TSL2561 driver functions implemented
 * in tsl2561.c */

/**
 * Initializes the TSL2561 sensor over I2C.
 * Powers off the sensor, resets I2C, then configures power and timing settings
 * in polling mode.
 *
 * @param i2c Pointer to the initialized XIic instance
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int tsl2561_init(XIic* i2c);

/**
 * Reads a channel (CH0 or CH1) from the TSL2561 sensor.
 * Sends the register address and reads 2 bytes (low and high) to form a 16-bit
 * value.
 *
 * @param i2c Pointer to the initialized XIic instance
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1)
 * @return 16-bit channel value on success, 0 on failure
 */
float tsl2561_readChannel(XIic* i2c, tsl2561_channel_t channel);

/**
 * Calculates lux value from CH0 and CH1 readings (optional).
 * Uses the TSL2561 datasheet formula to convert raw sensor data to lux (not
 * implemented yet).
 *
 * @param ch0 Channel 0 value (Visible + IR)
 * @param ch1 Channel 1 value (IR only)
 * @return Lux value as a float
 */
float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);

#endif /* TSL2561_H */
