/*
 * i2c.c - I2C driver implementation for AXI IIC controller on Nexys A7
 *
 * Purpose: Provides polling-based I2C functionality for integrating the TSL2561
 *          light sensor with a Microblaze processor running FreeRTOS. The driver
 *          is optimized for minimal memory usage and simplicity, avoiding
 *          interrupt-driven complexity where possible.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Requires Xilinx I2C driver (XIic) and definitions from i2c.h.
 */

#include "i2c.h"

/*--------------------------------------------------------------------------*/
/* Global Variables                                                         */
/*--------------------------------------------------------------------------*/
/**
 * @brief Pointer to I2C configuration data retrieved during initialization.
 */
XIic_Config* ConfigPtr;

/**
 * @brief Shared interrupt controller instance defined in main.c.
 * @note Included for compatibility but unused in this polling-based driver.
 */
extern XIntc Intc;

/*--------------------------------------------------------------------------*/
/* Function Definitions                                                     */
/*--------------------------------------------------------------------------*/
/**
 * i2c_init - Initializes the AXI IIC controller in polling mode
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 *
 * Description: Configures the AXI IIC peripheral as an I2C master with minimal
 *              reset operations. Sets the default slave address to the TSL2561
 *              sensor (0x39). Uses polling instead of interrupts for simplicity.
 */
int i2c_init ( void )
{
    int status;

    /* Lookup I2C configuration based on device ID */
    ConfigPtr = XIic_LookupConfig ( IIC_DEVICE_ID );
    if ( !ConfigPtr )
    {
        xil_printf ( "[ERROR] I2C config lookup failed\r\n" );
        return XST_FAILURE;
    }

    /* Initialize I2C instance with configuration data */
    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C initialization failed: %d\r\n", status );
        return XST_FAILURE;
    }

    /* Perform a minimal reset and enable I2C as master */
    XIic_WriteReg ( IicInstance.BaseAddress, 0x40, 0xA );   // Write 0xA to reset register
    usleep ( 20000 );                                       // Wait 20ms for reset to complete
    XIic_WriteReg ( IicInstance.BaseAddress, 0x100, 0x81 ); // Enable (bit 0) + Master mode (bit 7)

    /* Start the I2C controller */
    status = XIic_Start ( &IicInstance );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C start failed: %d\r\n", status );
        return XST_FAILURE;
    }

    /* Set the default slave address to TSL2561 (0x39) */
    status = XIic_SetAddress ( &IicInstance, XII_ADDR_TO_SEND_TYPE, I2C_SLAVE_ADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Failed to set I2C slave address: %d\r\n", status );
        return XST_FAILURE;
    }

    return XST_SUCCESS; // Initialization successful
}

/**
 * i2c_scan - Scans the I2C bus for devices
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 *
 * Description: Probes I2C addresses from 0x00 to 0x77 (excluding reserved ranges)
 *              using a 0-byte write to detect device ACKs. Prints results with
 *              minimal memory overhead. Waits for bus idle state before starting.
 */
void i2c_scan ( XIic* InstancePtr )
{
    int     devices_found   = 0;        // Counter for detected devices
    uint8_t probe_data[ 1 ] = { 0x00 }; // Minimal buffer for probing (0-byte write)
    int     status;

    /* Wait for I2C bus to become idle with a timeout */
    int timeout = 100000; // Arbitrary timeout value (100,000 iterations)
    while ( XIic_IsIicBusy ( InstancePtr ) && --timeout > 0 )
        ;
    if ( timeout <= 0 )
    {
        xil_printf ( "[ERROR] I2C bus timeout during scan\r\n" );
        return;
    }

    xil_printf ( "Scanning I2C bus...\r\n" );

    /* Iterate through valid I2C addresses (0x00 to 0x77) */
    for ( uint8_t addr = 0x00; addr <= 0x77; addr++ )
    {
        /* Skip reserved address ranges: 0x00-0x07 (general call) and 0x78-0x7F (10-bit addressing)
         */
        if ( ( addr & 0xF8 ) == 0 || ( addr & 0xF8 ) == 0x78 )
            continue;

        /* Set the target slave address */
        status = XIic_SetAddress ( InstancePtr, XII_ADDR_TO_SEND_TYPE, addr );
        if ( status == XST_SUCCESS )
        {
            /* Send a 0-byte write to probe for an ACK */
            status = XIic_MasterSend ( InstancePtr, probe_data, 0 );
            if ( status == XST_SUCCESS )
            {
                devices_found++;
                xil_printf ( "Device found at address 0x%02X\r\n", addr );
            }
        }
        usleep ( 1000 ); // 1ms delay to ensure bus stability between probes
    }

    xil_printf ( "Scan complete. Found %d device(s).\r\n", devices_found );
}

/**
 * i2c_soft_reset - Performs a soft reset of the I2C peripheral
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 * @return XST_SUCCESS if reset succeeds, XST_FAILURE if bus remains busy
 *
 * Description: Resets the AXI IIC controller by writing to its reset register,
 *              then re-enables it as a master. Includes minimal delays for
 *              stability and checks bus status post-reset.
 */
int i2c_soft_reset ( XIic* InstancePtr )
{
    /* Write reset value to reset register */
    XIic_WriteReg ( InstancePtr->BaseAddress, 0x40, 0xA ); // Reset command (0xA)
    usleep ( 20000 );                                      // Wait 20ms for reset to complete

    /* Re-enable I2C as master */
    XIic_WriteReg ( InstancePtr->BaseAddress, 0x100, 0x81 ); // Enable (bit 0) + Master mode (bit 7)
    usleep ( 5000 );                                         // Wait 5ms for stabilization

    /* Check if bus is still busy after reset */
    if ( XIic_IsIicBusy ( InstancePtr ) )
    {
        xil_printf ( "[ERROR] I2C bus busy after soft reset\r\n" );
        return XST_FAILURE;
    }

    return XST_SUCCESS; // Reset successful
}