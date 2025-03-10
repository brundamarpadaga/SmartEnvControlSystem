/*
 * main.c - Entry point for TSL2561 and BME280-based PID control system
 *
 * Purpose: This file initializes the FreeRTOS environment, sets up the TSL2561
 *          and BME280 sensors, and creates tasks for reading lux, temperature,
 *          pressure, and humidity values, running a PID controller to adjust
 *          LED brightness, and handling user input and display updates.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Final Project - Environmental Control System
 */

#include "main.h"

#include <stdlib.h>

/*
 * main - Program entry point
 * Returns: int (never returns in FreeRTOS application; -1 on initialization failure)
 *
 * Initializes the hardware, sensors, FreeRTOS synchronization primitives (semaphores
 * and queues), and tasks for the environmental control system. Starts the scheduler
 * to begin task execution.
 */
int main ( void )
{
    // Print startup message to console
    xil_printf ( "Hello from FreeRTOS LUX and BME280 PID Controller\r\n" );

    // Set up hardware components (GPIO, interrupts, etc.)
    prvSetupHardware ( );

    // Perform initial hardware and peripheral setup
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Hardware initialization failed\r\n" );
        return -1;  // Exit if initialization fails
    }

    // Initialize TSL2561 light sensor
    tsl2561_init ( &IicInstance );

    // Initialize BME280 environmental sensor
    if ( bme_init ( &IicInstance ) != 0 )
    {
        xil_printf ( "[ERROR] BME280 initialization failed\r\n" );
        return -1;  // Exit if BME280 init fails
    }

    // Read BME280 calibration data for accurate measurements
    if ( bme_read_calibration_data ( &IicInstance, &calib_data ) != 0 )
    {
        xil_printf ( "[ERROR] BME280 calibration data read failed\r\n" );
        return -1;  // Exit if calibration data read fails
    }

    // Create binary semaphore for GPIO interrupt handling
    vSemaphoreCreateBinary ( binary_sem );

    // Create binary semaphore for BME280 task synchronization
    bme280_sem = xSemaphoreCreateBinary ( );
    if ( bme280_sem == NULL )
    {
        xil_printf ( "[ERROR] BME280 semaphore creation failed\r\n" );
        return -1;  // Exit if semaphore creation fails
    }
    xSemaphoreGive ( bme280_sem );  // Initially release semaphore

    // Create queues for PID task communication
    toPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );  // Queue for input data to PID
    fromPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint32_t ) );  // Queue for PID output data
    configASSERT ( toPID );    // Assert queue creation success
    configASSERT ( fromPID );  // Assert queue creation success

    // Static PID structure for LED brightness control
    static PID_t ledPID;

    // Create FreeRTOS tasks with appropriate priorities and stack sizes
    xTaskCreate ( Parse_Input_Task,
                  "Parse_Input",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  1,
                  NULL );  // User input task
    xTaskCreate (
        PID_Task, "PID", configMINIMAL_STACK_SIZE, &ledPID, 1, NULL );  // PID control task
    xTaskCreate (
        Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL );  // Display update task
    xTaskCreate ( BME280_Task,
                  "BME280",
                  configMINIMAL_STACK_SIZE * 2,
                  NULL,
                  1,
                  NULL );  // BME280 sensor task (larger stack)

    // Start the FreeRTOS scheduler to begin task execution
    xil_printf ( "Starting the scheduler\r\n" );
    vTaskStartScheduler ( );

    // Should never reach here in a FreeRTOS application; indicate failure if it does
    return -1;
}

/*
 * prvSetupHardware - Configure hardware peripherals
 *
 * Sets up GPIO for buttons and switches, including interrupt handling, to enable
 * user input for the control system.
 */
static void prvSetupHardware ( void )
{
    uint32_t            xStatus;  // Status variable for initialization and interrupt setup
    const unsigned char ucSetToInput = 0xFFU;  // All GPIO pins set as inputs

    xil_printf ( "Initializing GPIO's\r\n" );

    // Initialize GPIO instance for buttons and switches
    xStatus = XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );
    if ( xStatus == XST_SUCCESS )
    {
        // Install interrupt handler for GPIO
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL );
        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );
            // Set GPIO channels as inputs for buttons and switches
            XGpio_SetDataDirection ( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
            XGpio_SetDataDirection ( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );
            // Enable interrupts for GPIO
            vPortEnableInterrupt ( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR );
            XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
        }
    }
    configASSERT ( ( xStatus == pdPASS ) );  // Assert successful hardware setup
}

/*
 * gpio_intr - GPIO interrupt service routine
 * @pvUnused: Unused parameter (required by ISR signature)
 *
 * Handles GPIO interrupts from button presses, signals the input task via semaphore,
 * and clears the interrupt.
 */
static void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Flag for context switch necessity
    // Signal input task that an interrupt occurred
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    // Clear GPIO interrupt
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
    // Yield to higher-priority task if needed
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
}

/*
 * do_init - Initialize additional hardware components
 * Returns: XST_SUCCESS on success, XST_FAILURE on failure
 *
 * Configures the NX4IO board, I2C controller, and RGB LED for system operation.
 */
int do_init ( void )
{
    int status;

    // Initialize NX4IO board (display and LEDs)
    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        return XST_FAILURE;  // Return failure if NX4IO init fails
    }

    // Look up I2C configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        return XST_FAILURE;  // Return failure if config lookup fails
    }

    // Initialize I2C controller with configuration
    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        return status;  // Return failure if I2C init fails
    }

    // Start I2C controller
    XIic_Start ( &IicInstance );
    // Enable blue channel of RGB LED (initially off)
    NX4IO_RGBLED_setChnlEn ( RGB1, false, false, true );
    return XST_SUCCESS;  // Success: Hardware initialized
}

/*
 * Parse_Input_Task - Task to process user input from buttons and switches
 * @p: Task parameters (unused)
 *
 * Monitors GPIO inputs via semaphore, reads button and switch states, and sends
 * combined data to the PID task via a queue. Updates LEDs to reflect switch states.
 */
void Parse_Input_Task ( void* p )
{
    uint8_t  btns        = 0x00;    // Button states (masked to 4 bits)
    uint8_t  sws         = 0x00;    // Switch states (8 bits)
    uint16_t ValueToSend = 0x0000;  // Combined button/switch value for queue

    while ( 1 )
    {
        // Wait for GPIO interrupt with 500ms timeout
        if ( xSemaphoreTake ( binary_sem, 500 ) )
        {
            btns = ( NX4IO_getBtns ( ) & 0x1E );                  // Read buttons, mask to bits 1-4
            sws  = (uint8_t) ( NX4IO_getSwitches ( ) & 0x00FF );  // Read switches, mask to 8 bits
            ValueToSend |= ( ( btns << 8 ) | sws );  // Combine: btns in high byte, sws in low byte
            NX4IO_setLEDs ( sws );                   // Reflect switch states on LEDs
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );  // Send to PID task
            ValueToSend &= 0x0000;                               // Reset value for next iteration
        }
    }
}

/*
 * PID_Task - Task to implement PID control for LED brightness
 * @p: Pointer to PID_t structure for control parameters
 *
 * Reads lux values from TSL2561, processes user input to adjust PID parameters
 * and setpoint, calculates PID output, and adjusts LED brightness accordingly.
 * Sends data to display task via queue.
 */
void PID_Task ( void* p )
{
    PID_t*      pid     = (PID_t*) p;           // PID structure pointer
    float       pidOUT  = 0;                    // PID controller output
    uint16_t    tsl2561 = 0;                    // Lux value from TSL2561
    uint8_t     pwmLED  = 127;                  // PWM duty cycle for LED (initially 50%)
    uint16_t    btnSws;                         // Combined button/switch input from queue
    uint32_t    setpntLux;                      // Combined setpoint and lux for display
    uint8_t     btns, sws;                      // Separated button and switch states
    float       baseID = 0.01, baseP = 0.1;     // Base increments for Ki/Kd and Kp
    uint8_t     baseSP        = 1, incScaling;  // Base setpoint increment and scaling factor
    uint8_t     btnPrint      = 0x00;           // Toggle for printing PID values
    TickType_t  lastTick      = xTaskGetTickCount ( );  // Last time for delta calculation
    static bool isInitialized = false;                  // PID initialization flag

    // Initialize PID parameters if not already done
    if ( !isInitialized )
    {
        isInitialized = pid_init ( pid );
    }

    while ( 1 )
    {
        // Receive input from Parse_Input_Task with 42ms timeout
        if ( xQueueReceive ( toPID, &btnSws, 42 ) == pdPASS )
        {
            btns = ( btnSws & 0x1E00 ) >> 8;  // Extract buttons (bits 9-12)
            sws  = ( btnSws & 0x0FF );        // Extract switches (bits 0-7)
        }
        else
        {
            btns = 0x00;  // No input received
        }

        // Toggle print mode with button 1 (bit 1)
        if ( btns & 0x02 ) btnPrint = !btnPrint;

        // Determine increment scaling based on switches 4-5 (bits 4-5)
        if ( !( sws & 0x30 ) )
            incScaling = 1;
        else if ( sws & 0x20 )
            incScaling = 10;
        else if ( sws & 0x10 )
            incScaling = 5;

        // Adjust setpoint or PID gains based on switch 3 (bit 3)
        if ( sws & 0x08 )
        {
            if ( btns & 0x08 )  // Button 3: Increase setpoint
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, true );
            else if ( btns & 0x04 )  // Button 2: Decrease setpoint
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, false );
        }
        else
        {
            // Adjust PID gains based on switches 6-7 (bits 6-7)
            switch ( sws & 0xC0 )
            {
                case 0x40:  // Adjust Kp
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, false );
                    break;
                case 0x80:  // Adjust Ki
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, false );
                    break;
                case 0xC0:  // Adjust Kd
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, false );
                    break;
            }
        }

        // Read TSL2561 channels and calculate lux (simplified formula)
        float ch0 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_0 );  // Broadband
        float ch1 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_1 );  // Infrared
        tsl2561   = (uint16_t) ch0 - ( (uint16_t) ch1 * 0.5 );  // Approximate lux value

        // Calculate time delta for PID (in seconds)
        TickType_t currentTick = xTaskGetTickCount ( );
        pid->delta_t           = ( ( currentTick - lastTick ) * ( 1 / 100.0f ) );
        lastTick               = currentTick;

        // Compute PID output based on current lux and switch settings
        pidOUT = pid_funct ( pid, tsl2561, sws );

        // Update PWM duty cycle with saturation limits
        if ( ( pwmLED + ( pidOUT * max_duty ) ) >= max_duty )
            pwmLED = max_duty;
        else if ( ( pwmLED + ( pidOUT * max_duty ) ) <= min_duty )
            pwmLED = min_duty;
        else
            pwmLED = (uint8_t) ( pwmLED + ( pidOUT * max_duty ) );

        // Apply PWM to blue channel of RGB LED
        NX4IO_RGBLED_setDutyCycle ( RGB1, 0, 0, pwmLED );

        // Print debug info if enabled
        if ( btnPrint )
        {
            xil_printf ( "Setpoint Value: %d\r\n", pid->setpoint );
            xil_printf ( "Lux Value: %d\r\n", tsl2561 );
            xil_printf ( "PWM LED Duty Cycle: %d\r\n", pwmLED );
        }

        // Send setpoint and lux to display task
        setpntLux = ( tsl2561 << 0 ) | ( pid->setpoint << 16 );
        xQueueSend ( fromPID, &setpntLux, mainDONT_BLOCK );

        // Print PID gains if button 4 is pressed
        if ( btns & 0x10 ) print_pid ( pid );
    }
}

/*
 * Display_Task - Task to update 7-segment display with PID data
 * @p: Task parameters (unused)
 *
 * Receives lux and setpoint values from PID task via queue and updates the
 * NX410 7-segment display with these values.
 */
void Display_Task ( void* p )
{
    uint32_t recievedLux;              // Combined setpoint and lux from queue
    uint16_t setpnt = 0x0000, luxVal;  // Separated setpoint and lux values

    while ( 1 )
    {
        // Wait indefinitely for data from PID task
        xQueueReceive ( fromPID, &recievedLux, portMAX_DELAY );
        luxVal = ( recievedLux & lux_mask );            // Extract lux (lower 16 bits)
        setpnt = ( ( recievedLux >> 16 ) & lux_mask );  // Extract setpoint (upper 16 bits)
        // Update high display with setpoint (3 digits)
        NX410_SSEG_setAllDigits ( SSEGHI,
                                  ( setpnt / 100 ),
                                  ( ( setpnt % 100 ) / 10 ),
                                  ( ( setpnt % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
        // Update low display with lux (3 digits)
        NX410_SSEG_setAllDigits ( SSEGLO,
                                  ( luxVal / 100 ),
                                  ( ( luxVal % 100 ) / 10 ),
                                  ( ( luxVal % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
    }
}

/*
 * pid_init - Initialize PID controller parameters
 * @pid: Pointer to PID_t structure to initialize
 * Returns: true on success
 *
 * Sets initial values for PID gains, setpoint, and limits for controlling LED brightness.
 */
bool pid_init ( PID_t* pid )
{
    pid->Kp         = 0.7;    // Proportional gain
    pid->Ki         = 0.2;    // Integral gain
    pid->Kd         = 0;      // Derivative gain
    pid->setpoint   = 100;    // Initial target lux value
    pid->integral   = 0;      // Accumulated integral term
    pid->prev_error = 0;      // Previous error for derivative
    pid->delta_t    = 0.437;  // Initial time delta (seconds)
    pid->max_lim    = 1000;   // Maximum setpoint/limit
    pid->min_lim    = 0;      // Minimum setpoint/limit
    return true;              // Success
}

/*
 * pid_funct - Compute PID controller output
 * @pid: Pointer to PID_t structure with control parameters
 * @lux_value: Current lux reading from TSL2561
 * @switches: Switch states to enable/disable P/I/D terms
 * Returns: Normalized PID output (fraction of setpoint)
 *
 * Calculates PID output using proportional, integral, and derivative terms,
 * with optional disabling via switches 0-2.
 */
float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches )
{
    uint16_t max_int = 2048;                       // Maximum integral limit
    float    min_int = -2048;                      // Minimum integral limit
    float    error   = pid->setpoint - lux_value;  // Current error

    // Proportional term (enabled by switch 0)
    float Pterm = ( switches & 0x01 ) ? pid->Kp * error : 0;

    // Integral term with saturation
    pid->integral += ( error * pid->delta_t );
    if ( pid->integral >= max_int )
        pid->integral = max_int;
    else if ( pid->integral <= min_int )
        pid->integral = min_int;
    float Iterm = ( switches & 0x02 ) ? pid->Ki * pid->integral : 0;

    // Derivative term (enabled by switch 2)
    float Dterm =
        ( switches & 0x04 ) ? pid->Kd * ( ( error - pid->prev_error ) / pid->delta_t ) : 0;
    pid->prev_error = error;  // Update previous error

    // Return normalized output
    return ( Pterm + Iterm + Dterm ) / (float) pid->setpoint;
}

/*
 * print_pid - Print PID gains to console
 * @pid: Pointer to PID_t structure with control parameters
 *
 * Displays current PID gains (Kp, Ki, Kd) with integer and decimal parts for debugging.
 */
void print_pid ( PID_t* pid )
{
    xil_printf ( "PID gains:\r\n" );
    xil_printf ( "Kp = %u.%02u\r\n",
                 (uint16_t) pid->Kp,
                 (uint16_t) ( ( pid->Kp - (uint16_t) pid->Kp ) * 100 ) );
    xil_printf ( "Ki = %u.%02u\r\n",
                 (uint16_t) pid->Ki,
                 (uint16_t) ( ( pid->Ki - (uint16_t) pid->Ki ) * 100 ) );
    xil_printf ( "Kd = %u.%02u\r\n",
                 (uint16_t) pid->Kd,
                 (uint16_t) ( ( pid->Kd - (uint16_t) pid->Kd ) * 100 ) );
}