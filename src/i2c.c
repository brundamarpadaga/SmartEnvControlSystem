
/*
 * i2c.c - I2C driver implementation for AXI IIC controller on Nexys A7
 *
 * Purpose: Provides polling-based I2C functionality for TSL2561 integration on
 * Microblaze with FreeRTOS. Optimized for minimal memory usage.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "i2c.h"

/* Global variables */
XIic_Config* ConfigPtr;  // Pointer to I2C configuration data
extern XIntc Intc;       // Shared interrupt controller instance (defined in main.c)

/**
 * Initializes the AXI IIC controller in polling mode with minimal reset.
 * Configures I2C as master and sets default slave address.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int i2c_init ( void )
{
    int status;

    /* Lookup I2C configuration */
    ConfigPtr = XIic_LookupConfig ( IIC_DEVICE_ID );
    if ( !ConfigPtr ) return XST_FAILURE;

    /* Initialize I2C instance */
    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS ) return XST_FAILURE;

    /* Reset and enable I2C as master */
    XIic_WriteReg ( IicInstance.BaseAddress, 0x40, 0xA );    // Reset
    vTaskDelay(pdMS_TO_TICKS(20));                           // 20ms reset timeout
    XIic_WriteReg ( IicInstance.BaseAddress, 0x100, 0x81 );  // Master + Enable

    /* Start I2C controller */
    status = XIic_Start ( &IicInstance );
    if ( status != XST_SUCCESS ) return XST_FAILURE;

    /* Set default slave address (TSL2561 at 0x39) */
    status = XIic_SetAddress ( &IicInstance, XII_ADDR_TO_SEND_TYPE, I2C_SLAVE_ADDR );
    if ( status != XST_SUCCESS ) return XST_FAILURE;

    return XST_SUCCESS;
}

/**
 * Scans the I2C bus for devices from 0x00 to 0x77 with minimal overhead.
 * Probes each address with a 0-byte write and checks ACK.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 */
void i2c_scan ( XIic* InstancePtr )
{
    int     devices_found   = 0;
    uint8_t probe_data[ 1 ] = { 0x00 };
    int     status;

    /* Wait for bus to become idle */
    int timeout = 100000;
    while ( XIic_IsIicBusy ( InstancePtr ) && --timeout > 0 );
    if ( timeout <= 0 ) return;

    for ( uint8_t addr = 0x00; addr <= 0x77; addr++ )
    {
        /* Skip reserved address ranges (0x00-0x07 and 0x78-0x7F) */
        if ( ( addr & 0xF8 ) == 0 || ( addr & 0xF8 ) == 0x78 ) continue;

        /* Set address and probe */
        status = XIic_SetAddress ( InstancePtr, XII_ADDR_TO_SEND_TYPE, addr );
        if ( status == XST_SUCCESS )
        {
            status = XIic_MasterSend ( InstancePtr, probe_data, 0 );
            if ( status == XST_SUCCESS )
            {
                devices_found++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Minimal delay for bus stability
    }
}

/**
 * Performs a soft reset of the I2C peripheral with minimal overhead.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 * @return XST_SUCCESS if reset succeeds, XST_FAILURE if bus remains busy
 */
int i2c_soft_reset ( XIic* InstancePtr )
{
    XIic_WriteReg ( InstancePtr->BaseAddress, 0x40, 0xA );    // Reset
    vTaskDelay(pdMS_TO_TICKS(20));                            // 20ms reset timeout
    XIic_WriteReg ( InstancePtr->BaseAddress, 0x100, 0x81 );  // Master + Enable
    vTaskDelay(pdMS_TO_TICKS(5));

    return XIic_IsIicBusy ( InstancePtr ) ? XST_FAILURE : XST_SUCCESS;
}
