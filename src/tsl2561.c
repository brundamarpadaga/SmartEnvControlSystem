/*
 * tsl2561.c - Driver implementation for TSL2561 light sensor
 *
 * Purpose: Provides functions to initialize and read data from the TSL2561
 *          ambient light sensor via I2C in a FreeRTOS-based system. Supports
 *          reading broadband (Channel 0) and infrared (Channel 1) light levels.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Requires Xilinx I2C driver (XIic), FreeRTOS for semaphore
 *               synchronization, and definitions from tsl2561.h and main.h.
 */

#include "tsl2561.h"

#include "main.h" // For i2c_sem, IicInstance, and other global definitions

/*--------------------------------------------------------------------------*/
/* External Declarations                                                    */
/*--------------------------------------------------------------------------*/
/**
 * @brief External function to perform a soft reset of the I2C peripheral.
 *        Defined in i2c.c.
 */
extern int i2c_soft_reset ( XIic* InstancePtr );

/*--------------------------------------------------------------------------*/
/* Function Definitions                                                     */
/*--------------------------------------------------------------------------*/
/**
 * tsl2561_init - Initialize the TSL2561 light sensor
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @return XST_SUCCESS on successful initialization, XST_FAILURE otherwise
 *
 * Description: Configures the TSL2561 sensor by powering it on and setting the
 *              integration time to 402ms with 16x gain. Uses a semaphore to
 *              ensure exclusive I2C access and includes a reset and timeout
 *              mechanism for robustness.
 */
int tsl2561_init ( XIic* i2c )
{
    uint8_t send[ 2 ]; // Buffer for I2C command and data
    int     status;

    /* Acquire I2C semaphore with 100ms timeout */
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
    {
        /* Reset I2C peripheral and wait for stabilization */
        i2c_soft_reset ( i2c );
        usleep ( 20000 ); // 20ms delay post-reset

        /* Wait for I2C bus to become idle with timeout */
        int timeout = TIMEOUT_COUNTER; // Defined in tsl2561.h
        while ( XIic_IsIicBusy ( i2c ) && --timeout > 0 )
            ;
        if ( timeout <= 0 )
        {
            xil_printf ( "[ERROR] I2C bus busy timeout in tsl2561_init\r\n" );
            xSemaphoreGive ( i2c_sem );
            return XST_FAILURE;
        }

        /* Set TSL2561 I2C address */
        if ( XIic_SetAddress ( i2c, XII_ADDR_TO_SEND_TYPE, TSL2561_ADDR ) != XST_SUCCESS )
        {
            xil_printf ( "[ERROR] Failed to set TSL2561 I2C address\r\n" );
            xSemaphoreGive ( i2c_sem );
            return XST_FAILURE;
        }

        /* Power on the TSL2561 */
        send[ 0 ] = TSL2561_CMD_CONTROL; // Control register (0x00 | 0x80 for command bit)
        send[ 1 ] = TSL2561_POWER_ON;    // Power on value (0x03)
        status    = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
        if ( status != 2 ) // Expect 2 bytes sent (command + data)
        {
            xil_printf ( "[ERROR] Failed to power on TSL2561 (bytes sent: %d)\r\n", status );
            xSemaphoreGive ( i2c_sem );
            return XST_FAILURE;
        }

        /* Set timing to 402ms integration with 16x gain */
        send[ 0 ] = TSL2561_CMD_TIMING;       // Timing register (0x01 | 0x80 for command bit)
        send[ 1 ] = TSL2561_TIMING_402MS_16X; // 402ms, 16x gain (0x02)
        status    = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
        if ( status != 2 )
        {
            xil_printf ( "[ERROR] Failed to set TSL2561 timing (bytes sent: %d)\r\n", status );
            xSemaphoreGive ( i2c_sem );
            return XST_FAILURE;
        }

        xSemaphoreGive ( i2c_sem ); // Release I2C semaphore
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in tsl2561_init\r\n" );
        return XST_FAILURE;
    }

    /* Wait for initial integration time to complete */
    usleep ( 410000 ); // 410ms delay (>402ms integration time)
    return XST_SUCCESS;
}

/**
 * tsl2561_readChannel - Read light data from a TSL2561 channel
 *
 * @param i2c Pointer to the XIic instance for I2C communication
 * @param channel Channel to read (TSL2561_CHANNEL_0 for broadband, TSL2561_CHANNEL_1 for IR)
 * @return Light level as a float (raw ADC value), or 0 on failure
 *
 * Description: Reads the 16-bit raw ADC value from the specified TSL2561 channel
 *              (Channel 0: broadband, Channel 1: infrared). Uses a semaphore for
 *              I2C access and restarts the I2C controller to ensure a clean read.
 */
float tsl2561_readChannel ( XIic* i2c, tsl2561_channel_t channel )
{
    uint8_t buf[ 2 ]; // Buffer for register address and received data
    int     status;

    /* Acquire I2C semaphore with 500ms timeout (increased for reliability) */
    if ( xSemaphoreTake ( i2c_sem, pdMS_TO_TICKS ( 500 ) ) == pdTRUE )
    {
        /* Select register based on channel */
        buf[ 0 ] = ( channel == TSL2561_CHANNEL_0 ? DATA0LOW_REG : DATA1LOW_REG ) |
                   0x80; // Command bit set

        /* Restart I2C controller for a clean transaction */
        XIic_Stop ( i2c );
        usleep ( 5000 ); // 5ms delay for stabilization
        status = XIic_Start ( i2c );
        if ( status != XST_SUCCESS )
        {
            xil_printf ( "[ERROR] I2C restart failed before read (status: %d)\r\n", status );
            xSemaphoreGive ( i2c_sem );
            return 0;
        }

        /* Send register address to read from */
        status = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START );
        if ( status != 1 ) // Expect 1 byte sent (register address)
        {
            xil_printf (
                "[ERROR] Send address 0x%02X failed (bytes sent: %d)\r\n", buf[ 0 ], status );
            xSemaphoreGive ( i2c_sem );
            return 0;
        }

        /* Read 2 bytes of data (low and high) */
        status = XIic_Recv ( i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP );
        if ( status != 2 ) // Expect 2 bytes received
        {
            xil_printf (
                "[ERROR] Receive failed for channel %d (bytes read: %d)\r\n", channel, status );
            xSemaphoreGive ( i2c_sem );
            return 0;
        }

        xSemaphoreGive ( i2c_sem ); // Release I2C semaphore

        /* Combine low and high bytes into 16-bit value */
        uint16_t value = ( buf[ 1 ] << 8 ) | buf[ 0 ];
        return (float) value; // Return raw ADC value as float
    }
    else
    {
        xil_printf ( "[ERROR] I2C semaphore timeout in tsl2561_readChannel\r\n" );
        return 0;
    }
}