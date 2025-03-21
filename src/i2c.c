/*
 * ===========================================================================
 * File:    i2c.c
 * Summary: AXI IIC Controller Driver for Nexys A7 in Polling Mode
 * ===========================================================================
 * Purpose: This file implements a lightweight, polling-based I2C driver for the
 *          AXI IIC controller on the Nexys A7 board, integrated with Microblaze
 *          and FreeRTOS. It provides initialization, bus scanning, and soft reset
 *          functionality optimized for minimal memory usage, primarily for
 *          interfacing with the TSL2561 light sensor.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 *
 * Dependencies:
 *   - i2c.h: Header file defining function prototypes and constants
 *   - main.h: Provides shared I2C instance (IicInstance) and interrupt controller
 *   - XIic: Xilinx I2C driver for low-level register access
 *   - FreeRTOS: Used in the broader system context (not directly here)
 *
 * Notes:
 *   - Operates in polling mode to avoid interrupt overhead.
 *   - Default slave address is set to 0x39 (TSL2561).
 *   - Bus scanning skips reserved I2C address ranges (0x00-0x07, 0x78-0x7F).
 */

/* ===========================================================================
 * Includes
 * ===========================================================================
 */
#include "i2c.h"    // I2C-specific definitions and prototypes
#include "main.h"   // System-wide definitions (IicInstance, etc.)
#include <unistd.h> // For usleep (microsecond delays)

/* ===========================================================================
 * Global Variables
 * ===========================================================================
 */
XIic_Config* ConfigPtr; // Pointer to I2C configuration data, populated during init
extern XIntc Intc;      // Shared interrupt controller instance (defined in main.c)

/* ===========================================================================
 * Function Definitions
 * ===========================================================================
 */

/**
 * @brief Initializes the AXI IIC controller in polling mode.
 * @details Configures the I2C peripheral as a master, performs a minimal reset,
 *          and sets the default slave address to the TSL2561 (0x39). Uses polling
 *          to minimize memory usage and avoid interrupt overhead.
 * @return int XST_SUCCESS on successful initialization, XST_FAILURE otherwise.
 */
int i2c_init(void)
{
    int status; // Status variable for I2C operations

    /* Lookup I2C configuration based on device ID */
    ConfigPtr = XIic_LookupConfig(IIC_DEVICE_ID);
    if (!ConfigPtr)
        return XST_FAILURE; // Return failure if config not found

    /* Initialize I2C instance with configuration data */
    status = XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
    if (status != XST_SUCCESS)
        return XST_FAILURE; // Return failure if init fails

    /* Perform minimal reset and enable I2C as master */
    XIic_WriteReg(IicInstance.BaseAddress, 0x40, 0xA); // Write reset bit to control reg
    usleep(20000);                                     // Wait 20ms for reset to complete
    XIic_WriteReg(
        IicInstance.BaseAddress, 0x100, 0x81); // Enable master mode (bit 7) and I2C (bit 0)

    /* Start the I2C controller */
    status = XIic_Start(&IicInstance);
    if (status != XST_SUCCESS)
        return XST_FAILURE; // Return failure if start fails

    /* Set default slave address to TSL2561 (0x39) */
    status = XIic_SetAddress(&IicInstance, XII_ADDR_TO_SEND_TYPE, I2C_SLAVE_ADDR);
    if (status != XST_SUCCESS)
        return XST_FAILURE; // Return failure if address set fails

    return XST_SUCCESS; // Return success on completion
}

/**
 * @brief Scans the I2C bus for active devices.
 * @details Probes addresses from 0x00 to 0x77 using a 0-byte write to check for ACKs,
 *          skipping reserved ranges. Prints detected devices with minimal overhead.
 * @param InstancePtr Pointer to the initialized I2C instance.
 */
void i2c_scan(XIic* InstancePtr)
{
    int     devices_found = 0;      // Counter for detected devices
    uint8_t probe_data[1] = {0x00}; // Dummy data for probing (0-byte write)
    int     status;                 // Status variable for I2C operations

    /* Wait for I2C bus to become idle with timeout */
    int timeout = 100000; // Arbitrary large timeout value
    while (XIic_IsIicBusy(InstancePtr) && --timeout > 0)
        ;
    if (timeout <= 0)
        return; // Exit if bus remains busy

    /* Scan I2C addresses from 0x00 to 0x77 */
    for (uint8_t addr = 0x00; addr <= 0x77; addr++)
    {
        /* Skip reserved address ranges: 0x00-0x07 (general call) and 0x78-0x7F (10-bit addressing)
         */
        if ((addr & 0xF8) == 0 || (addr & 0xF8) == 0x78)
            continue;

        /* Set the current address to probe */
        status = XIic_SetAddress(InstancePtr, XII_ADDR_TO_SEND_TYPE, addr);
        if (status == XST_SUCCESS)
        {
            /* Attempt a 0-byte write to check for device acknowledgment */
            status = XIic_MasterSend(InstancePtr, probe_data, 0);
            if (status == XST_SUCCESS)
            {
                devices_found++; // Increment counter if device responds
                // Note: Original code doesn't print, but could add xil_printf here if desired
            }
        }
        usleep(1000); // 1ms delay for bus stability between probes
    }
    // Note: devices_found is computed but not used; could log total if needed
}

/**
 * @brief Performs a soft reset of the I2C peripheral.
 * @details Resets the I2C controller by writing to the control register, waits for
 *          stabilization, and re-enables master mode. Checks bus status post-reset.
 * @param InstancePtr Pointer to the initialized I2C instance.
 * @return int XST_SUCCESS if reset succeeds and bus is idle, XST_FAILURE if bus remains busy.
 */
int i2c_soft_reset(XIic* InstancePtr)
{
    XIic_WriteReg(InstancePtr->BaseAddress, 0x40, 0xA);   // Write reset bit to control reg
    usleep(20000);                                        // Wait 20ms for reset to complete
    XIic_WriteReg(InstancePtr->BaseAddress, 0x100, 0x81); // Re-enable master mode and I2C
    usleep(5000);                                         // Wait 5ms for stabilization

    /* Check if bus is idle after reset */
    return XIic_IsIicBusy(InstancePtr) ? XST_FAILURE : XST_SUCCESS;
}