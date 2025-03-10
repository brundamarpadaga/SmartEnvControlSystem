/*
 * tsl2561.c - Driver implementation for TSL2561 luminosity sensor
 *
 * Purpose: Initializes the TSL2561 sensor and reads its channels (CH0 and CH1)
 * over I2C in polling mode. Optimized for minimal memory usage on Nexys A7
 * FPGA, Microblaze, and FreeRTOS.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "tsl2561.h"

/* Forward declarations from i2c.c */
extern int i2c_soft_reset ( XIic* InstancePtr );

/**
 * Initializes the TSL2561 sensor over I2C with minimal footprint.
 * Powers on and configures timing, relying on I2C reset for stability.
 *
 * @param i2c Pointer to the initialized IIC instance
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int tsl2561_init ( XIic* i2c )
{
    uint8_t send[ 2 ];  // Buffer for I2C write commands (command + data)
    int     status;

    /* Reset I2C to ensure clean state */
    i2c_soft_reset ( i2c );
    usleep ( 20000 );  // 20ms reset timeout

    /* Wait for I2C bus to become idle */
    int timeout = TIMEOUT_COUNTER;
    while ( XIic_IsIicBusy ( i2c ) && --timeout > 0 );
    if ( timeout <= 0 ) return XST_FAILURE;

    /* Set TSL2561 I2C slave address (0x39) */
    if ( XIic_SetAddress ( i2c, XII_ADDR_TO_SEND_TYPE, TSL2561_ADDR ) != XST_SUCCESS )
        return XST_FAILURE;

    /* Power on the TSL2561 */
    send[ 0 ] = TSL2561_CMD_CONTROL;
    send[ 1 ] = TSL2561_POWER_ON;
    status    = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
    if ( status != 2 ) return XST_FAILURE;

    /* Configure timing: 402ms integration, 16x gain */
    send[ 0 ] = TSL2561_CMD_TIMING;
    send[ 1 ] = TSL2561_TIMING_402MS_16X;
    status    = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
    if ( status != 2 ) return XST_FAILURE;

    /* Delay 410ms for integration time */
    usleep ( 410000 );

    return XST_SUCCESS;
}

/**
 * Reads a channel (CH0 or CH1) from the TSL2561 sensor with minimal overhead.
 * Sends the register address and receives 2 bytes to form a 16-bit value.
 *
 * @param i2c Pointer to the initialized IIC instance
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1)
 * @return 16-bit channel value on success, 0 on failure
 */
float tsl2561_readChannel ( XIic* i2c, tsl2561_channel_t channel )
{
    uint8_t buf[ 2 ];  // Buffer for register address (1 byte) and data (2 bytes)
    int     status;    // Status return value for I2C operations

    /* Select the low byte register based on the channel */
    buf[ 0 ] =
        ( channel == TSL2561_CHANNEL_0 ? DATA0LOW_REG : DATA1LOW_REG ) | 0x80;  // Set command bit

    /* Restart I2C before read */
    XIic_Stop ( i2c );
    usleep ( 5000 );
    status = XIic_Start ( i2c );
    if ( status != XST_SUCCESS )
    {
        xil_printf (
            "       [ERROR] ---- I2C restart failed before read "
            "(status: %d) ----\r\n",
            status );
        return 0;
    }

    /* Send the register address with repeated start */
    status = XIic_Send ( i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START );
    if ( status != 1 )
    {
        xil_printf (
            "       [ERROR] ---- Send address 0x%02X failed (bytes "
            "sent: %d) ----\r\n",
            buf[ 0 ],
            status );
        return 0;
    }

    /* Receive 2 bytes (low and high) from the selected channel */
    status = XIic_Recv ( i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP );
    if ( status != 2 )
    {
        xil_printf (
            "       [ERROR] ---- Receive failed for channel %d (bytes "
            "read: %d) ----\r\n",
            channel,
            status );
        return 0;
    }

    /* Combine high and low bytes into a 16-bit value */
    uint16_t value = ( buf[ 1 ] << 8 ) | buf[ 0 ];
    return value;
}
