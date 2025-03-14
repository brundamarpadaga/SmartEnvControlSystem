
#ifndef I2C_H
#define I2C_H

/*
 * i2c.h - Header file for I2C driver implementation on Nexys A7
 *
 * Purpose: Defines constants, global variables, and function prototypes for
 *          initializing and managing the AXI IIC controller in polling mode.
 *          Supports communication with the TSL2561 sensor and other I2C devices
 *          within a FreeRTOS environment on a Microblaze system.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* BSP includes - Necessary Xilinx BSP headers for hardware interaction */
#include "sleep.h"        // Provides usleep() for timing delays
#include "xiic.h"         // Xilinx I2C driver API for AXI IIC controller
#include "xil_printf.h"   // Xilinx printf implementation for debug output
#include "xintc.h"        // Xilinx interrupt controller API (used for interrupt setup)
#include "xparameters.h"  // Hardware-specific parameters from the BSP

/* FreeRTOS includes - Core FreeRTOS headers for task and interrupt management
 */
#include "FreeRTOS.h"  // Main FreeRTOS kernel definitions
#include "task.h"      // Task management APIs (e.g., vTaskDelay)

/* Device instance definitions - Hardware-specific IDs and addresses */
#define IIC_DEVICE_ID \
    XPAR_IIC_0_DEVICE_ID  // Device ID for the AXI IIC controller, sourced
                          // from xparameters.h
#define INTC_DEVICE_ID \
    XPAR_INTC_0_DEVICE_ID  // Device ID for the AXI interrupt controller,
                           // sourced from xparameters.h
#define I2C_SLAVE_ADDR \
    0x39  // Default I2C slave address (0x39 for TSL2561 with ADDR pin
          // floating)

/* Global variables - Extern declarations for objects defined in i2c.c */
extern XIic IicInstance;        // IIC driver instance for interacting with the AXI IIC
                                // hardware
extern XIic_Config* ConfigPtr;  // Pointer to I2C configuration data, populated
                                // by XIic_LookupConfig
extern XIntc Intc;              // Shared interrupt controller instance, defined in main.c
                                // for system-wide interrupts

/* Function prototypes - Declarations for I2C driver functions implemented in
 * i2c.c */

/**
 * Initializes the AXI IIC controller in polling mode.
 * Sets up the controller as a master with the default slave address (0x39).
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int i2c_init ( void );

/**
 * Scans the I2C bus for devices from address 0x00 to 0x77.
 * Uses polling to probe each address and reports detected devices via
 * xil_printf.
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 */
void i2c_scan ( XIic* InstancePtr );

/**
 * Reads and displays the I2C Control Register (CR) contents.
 * Provides a bit-by-bit breakdown for debugging controller state.
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 */
void i2c_read_control ( XIic* InstancePtr );

/**
 * Reads and displays the I2C Status Register (SR) contents.
 * Provides a bit-by-bit breakdown with diagnostic notes for debugging bus
 * state.
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 */
void i2c_read_status ( XIic* InstancePtr );

/**
 * Performs a soft reset of the I2C peripheral.
 * Resets the controller, clears FIFOs, and re-enables I2C operation.
 *
 * @param InstancePtr Pointer to the initialized XIic instance
 * @return XST_SUCCESS if reset succeeds, XST_FAILURE if bus remains busy
 */
int i2c_soft_reset ( XIic* InstancePtr );

#endif /* I2C_H */