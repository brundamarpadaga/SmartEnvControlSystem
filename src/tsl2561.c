/*
 * ===========================================================================
 * File:    tsl2561.c
 * Summary: TSL2561 Light Sensor Driver for Environmental Control System
 * ===========================================================================
 * Purpose: This file implements functions to interface with the TSL2561 light
 *          sensor over I2C in a FreeRTOS environment. It provides initialization
 *          and channel reading capabilities for measuring luminosity (visible
 *          + IR and IR-only channels). The driver ensures thread-safe I2C access
 *          using semaphores and handles sensor power-on and timing configuration.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 *
 *
 * Dependencies:
 *   - tsl2561.h: Header file defining function prototypes and constants
 *   - main.h: Provides I2C instance, semaphores, and system-wide definitions
 *   - FreeRTOS: Semaphore and timing support
 *   - XIic: Xilinx I2C driver for low-level communication
 *
 * Notes:
 *   - The TSL2561 is configured for 402ms integration time.
 *   - Channel readings return raw 16-bit values as floats for compatibility.
 *   - I2C operations include timeouts and error logging for robustness.
 */

/* ===========================================================================
 * Includes
 * ===========================================================================
 */
#include "tsl2561.h" // TSL2561-specific definitions and prototypes
#include "main.h"    // System-wide definitions (I2C instance, semaphores, etc.)
#include <unistd.h>  // For usleep (microsecond delays)

/* ===========================================================================
 * External Declarations
 * ===========================================================================
 */
extern int i2c_soft_reset(XIic* InstancePtr); // Soft reset function from main system

/* ===========================================================================
 * Function Definitions
 * ===========================================================================
 */

/**
 * @brief Initializes the TSL2561 light sensor.
 * @details Performs a soft I2C reset, sets the sensor address, powers it on,
 *          and configures timing (402ms integration, 16x gain). Includes delays
 *          for stabilization and checks for I2C bus readiness.
 * @param i2c Pointer to the I2C instance.
 * @return int XST_SUCCESS on success, XST_FAILURE on any error (semaphore, I2C).
 */
int tsl2561_init(XIic* i2c)
{
    uint8_t send[2]; // Buffer for I2C command and data bytes
    int     status;  // Status variable for I2C operations

    // Attempt to acquire I2C bus semaphore with 100ms timeout
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        i2c_soft_reset(i2c); // Reset I2C bus to clear any previous state
        usleep(20000);       // Wait 20ms for reset to complete

        // Wait for I2C bus to become idle, with timeout
        int timeout = TIMEOUT_COUNTER;
        while (XIic_IsIicBusy(i2c) && --timeout > 0)
            ;
        if (timeout <= 0)
        {
            xSemaphoreGive(i2c_sem); // Release semaphore on timeout
            return XST_FAILURE;      // Return failure if bus remains busy
        }

        // Set TSL2561 I2C address for communication
        if (XIic_SetAddress(i2c, XII_ADDR_TO_SEND_TYPE, TSL2561_ADDR) != XST_SUCCESS)
        {
            xSemaphoreGive(i2c_sem); // Release semaphore on failure
            return XST_FAILURE;      // Return failure if address set fails
        }

        // Power on the sensor
        send[0] = TSL2561_CMD_CONTROL; // Control register command
        send[1] = TSL2561_POWER_ON;    // Power-on value
        status  = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP);
        if (status != 2) // Expect 2 bytes sent
        {
            xSemaphoreGive(i2c_sem); // Release semaphore on failure
            return XST_FAILURE;      // Return failure if send fails
        }

        // Configure timing: 402ms integration, 16x gain
        send[0] = TSL2561_CMD_TIMING;       // Timing register command
        send[1] = TSL2561_TIMING_402MS_16X; // Timing and gain settings
        status  = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP);
        if (status != 2) // Expect 2 bytes sent
        {
            xSemaphoreGive(i2c_sem); // Release semaphore on failure
            return XST_FAILURE;      // Return failure if send fails
        }

        xSemaphoreGive(i2c_sem); // Release I2C bus
    }
    else
    {
        // Log error if semaphore acquisition fails
        xil_printf("[ERROR] I2C semaphore timeout in tsl2561_init\r\n");
        return XST_FAILURE; // Return failure
    }

    usleep(410000);     // Wait 410ms for sensor to stabilize after power-on
    return XST_SUCCESS; // Return success
}

/**
 * @brief Reads a channel from the TSL2561 sensor.
 * @details Reads 16-bit data from either Channel 0 (visible + IR) or Channel 1
 *          (IR only). Performs an I2C restart, sends the register address, and
 *          receives the data, returning it as a float.
 * @param i2c Pointer to the I2C instance.
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1).
 * @return float Raw 16-bit sensor value as a float, or 0 on error.
 */
float tsl2561_readChannel(XIic* i2c, tsl2561_channel_t channel)
{
    uint8_t buf[2]; // Buffer for register address and received data
    int     status; // Status variable for I2C operations

    // Attempt to acquire I2C bus semaphore with 500ms timeout
    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(500)) == pdTRUE)
    {
        // Select register based on channel, set command bit (0x80)
        buf[0] = (channel == TSL2561_CHANNEL_0 ? DATA0LOW_REG : DATA1LOW_REG) | 0x80;

        XIic_Stop(i2c);           // Stop I2C to ensure clean restart
        usleep(5000);             // Wait 5ms for stop to settle
        status = XIic_Start(i2c); // Restart I2C
        if (status != XST_SUCCESS)
        {
            // Log error if restart fails
            xil_printf("[ERROR] I2C restart failed before read (status: %d)\r\n", status);
            xSemaphoreGive(i2c_sem); // Release semaphore
            return 0;                // Return 0 on failure
        }

        // Send register address with repeated start
        status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START);
        if (status != 1) // Expect 1 byte sent
        {
            // Log error if address send fails
            xil_printf("[ERROR] Send address 0x%02X failed (bytes sent: %d)\r\n", buf[0], status);
            xSemaphoreGive(i2c_sem); // Release semaphore
            return 0;                // Return 0 on failure
        }

        // Receive 2 bytes of data (low and high)
        status = XIic_Recv(i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP);
        if (status != 2) // Expect 2 bytes received
        {
            // Log error if receive fails
            xil_printf(
                "[ERROR] Receive failed for channel %d (bytes read: %d)\r\n", channel, status);
            xSemaphoreGive(i2c_sem); // Release semaphore
            return 0;                // Return 0 on failure
        }

        xSemaphoreGive(i2c_sem); // Release I2C bus
        // Combine low and high bytes into 16-bit value
        uint16_t value = (buf[1] << 8) | buf[0];
        return (float) value; // Return as float
    }
    else
    {
        // Log error if semaphore acquisition fails
        xil_printf("[ERROR] I2C semaphore timeout in tsl2561_readChannel\r\n");
        return 0; // Return 0 on failure
    }
}