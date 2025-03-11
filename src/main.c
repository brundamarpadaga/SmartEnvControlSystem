/*
 * main.c - Main application file for FreeRTOS-based sensor and display system
 *
 * Purpose: Initializes hardware and FreeRTOS resources, sets up sensors (TSL2561,
 *          BME280), OLED display, and a PID controller for LED brightness based
 *          on ambient light. Manages tasks for input parsing, PID control, display
 *          updates, and sensor readings on a Microblaze system with Nexys A7 board.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 * Dependencies: Requires Xilinx BSP (XGpio, XIic), FreeRTOS, and drivers for
 *               TSL2561, BME280, OLED (SSD1306), and NX4IO peripherals.
 */

#include "main.h"

#include <stdlib.h> // For abs() used in sensor data formatting

/*--------------------------------------------------------------------------*/
/* Global Variables                                                         */
/*--------------------------------------------------------------------------*/
/**
 * @brief GPIO instance for handling buttons and switches on Nexys A7.
 */
XGpio xInputGPIOInstance;

/**
 * @brief I2C instance for communication with TSL2561, BME280, and OLED.
 */
XIic IicInstance;

/**
 * @brief Calibration data structure for BME280 sensor.
 */
struct bme280_calib_data calib_data;

/**
 * @brief Binary semaphore for GPIO interrupt synchronization.
 */
SemaphoreHandle_t binary_sem;

/**
 * @brief Semaphore for synchronizing BME280 sensor access.
 */
SemaphoreHandle_t bme280_sem;

/**
 * @brief Semaphore for synchronizing OLED display access.
 */
SemaphoreHandle_t oled_sem;

/**
 * @brief Semaphore for synchronizing I2C bus access.
 */
SemaphoreHandle_t i2c_sem;

/**
 * @brief Queue for sending button/switch data to PID task.
 */
xQueueHandle toPID = NULL;

/**
 * @brief Queue for sending PID setpoint and lux values to display task.
 */
xQueueHandle fromPID = NULL;

/**
 * @brief Global structure to store sensor data (temperature, pressure, humidity, luminosity).
 */
SensorData_t sensor_data = { 0 };

/*--------------------------------------------------------------------------*/
/* FreeRTOS Assertion Handler                                               */
/*--------------------------------------------------------------------------*/
/**
 * vAssertCalled - FreeRTOS assertion failure handler
 *
 * @param file Name of the file where the assertion failed
 * @param line Line number of the assertion failure
 *
 * Description: Prints an error message with file and line number, disables
 *              interrupts, and enters an infinite loop to halt execution.
 */
void vAssertCalled ( const char* file, unsigned long line )
{
    xil_printf ( "[ERROR] FreeRTOS assertion failed in %s, line %lu\r\n", file, line );
    taskDISABLE_INTERRUPTS ( );
    while ( 1 )
        ; // Halt execution
}

/*--------------------------------------------------------------------------*/
/* Main Function                                                            */
/*--------------------------------------------------------------------------*/
/**
 * main - Entry point of the application
 *
 * @return 0 on success (never reached due to scheduler), -1 on initialization failure
 *
 * Description: Initializes hardware, sensors, display, and FreeRTOS resources
 *              (semaphores, queues, tasks), then starts the scheduler. Handles
 *              initialization failures with error messages.
 */
int main ( void )
{
    xil_printf ( "Hello from FreeRTOS LUX, BME280, and OLED Controller\r\n" );

    prvSetupHardware ( ); // Configure GPIO and interrupts
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Hardware initialization failed\r\n" );
        return -1;
    }

    /* Create I2C semaphore before sensor initialization */
    i2c_sem = xSemaphoreCreateBinary ( );
    if ( i2c_sem == NULL )
    {
        xil_printf ( "[ERROR] I2C semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( i2c_sem ); // Initially available

    /* Initialize TSL2561 light sensor */
    if ( tsl2561_init ( &IicInstance ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] TSL2561 initialization failed\r\n" );
        return -1;
    }

    /* Initialize BME280 environmental sensor and read calibration data */
    if ( bme_init ( &IicInstance ) != 0 )
    {
        xil_printf ( "[ERROR] BME280 initialization failed\r\n" );
        return -1;
    }
    if ( bme_read_calibration_data ( &IicInstance, &calib_data ) != 0 )
    {
        xil_printf ( "[ERROR] BME280 calibration data read failed\r\n" );
        return -1;
    }

    /* Initialize OLED display */
    if ( lcd_init ( &IicInstance ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] OLED initialization failed\r\n" );
        return -1;
    }

    /* Create additional semaphores */
    vSemaphoreCreateBinary ( binary_sem ); // For GPIO interrupts
    bme280_sem = xSemaphoreCreateBinary ( );
    if ( bme280_sem == NULL )
    {
        xil_printf ( "[ERROR] BME280 semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( bme280_sem ); // Initially available

    oled_sem = xSemaphoreCreateBinary ( );
    if ( oled_sem == NULL )
    {
        xil_printf ( "[ERROR] OLED semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( oled_sem ); // Initially available

    /* Create queues for PID communication */
    toPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) ); // Buttons/switches to PID
    if ( toPID == NULL )
    {
        xil_printf ( "[ERROR] Failed to create toPID queue\r\n" );
    }
    fromPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint32_t ) ); // PID to display
    if ( fromPID == NULL )
    {
        xil_printf ( "[ERROR] Failed to create fromPID queue\r\n" );
    }
    configASSERT ( toPID ); // Verify queue creation
    configASSERT ( fromPID );

    static PID_t ledPID; // PID controller instance for LED brightness

    /* Create FreeRTOS tasks */
    xTaskCreate ( Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL );
    xTaskCreate ( PID_Task, "PID", configMINIMAL_STACK_SIZE * 2, &ledPID, 1, NULL );
    xTaskCreate ( Display_Task, "Disp", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL );
    xTaskCreate ( BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL );
    xTaskCreate ( LCD_Task, "LCD", configMINIMAL_STACK_SIZE * 4, &sensor_data, 1, NULL );

    xil_printf ( "Starting the scheduler\r\n" );
    vTaskStartScheduler ( ); // Start FreeRTOS scheduler

    return -1; // Should never reach here
}

/*--------------------------------------------------------------------------*/
/* Hardware Setup Function                                                  */
/*--------------------------------------------------------------------------*/
/**
 * prvSetupHardware - Configure GPIO and interrupt hardware
 *
 * Description: Initializes GPIO for buttons and switches, sets them as inputs,
 *              and configures an interrupt handler for button presses.
 */
void prvSetupHardware ( void )
{
    uint32_t            xStatus;
    const unsigned char ucSetToInput = 0xFFU; // All pins as inputs

    xil_printf ( "Initializing GPIO's\r\n" );
    xStatus = XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );
    if ( xStatus == XST_SUCCESS )
    {
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL );
        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );
            XGpio_SetDataDirection ( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput ); // Buttons
            XGpio_SetDataDirection ( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );  // Switches
            vPortEnableInterrupt ( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR );
            XGpio_InterruptEnable ( &xInputGPIOInstance,
                                    XGPIO_IR_CH1_MASK ); // Enable btn interrupts
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
        }
    }
    configASSERT ( ( xStatus == pdPASS ) ); // Verify setup success
}

/*--------------------------------------------------------------------------*/
/* Interrupt Handler                                                        */
/*--------------------------------------------------------------------------*/
/**
 * gpio_intr - GPIO interrupt handler for button presses
 *
 * @param pvUnused Unused parameter required by interrupt handler signature
 *
 * Description: Signals the binary semaphore from ISR context to wake the
 *              Parse_Input_Task, clears the interrupt, and yields if necessary.
 */
void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken ); // Yield if a higher-priority task was woken
}

/*--------------------------------------------------------------------------*/
/* Initialization Function                                                  */
/*--------------------------------------------------------------------------*/
/**
 * do_init - Initialize NX4IO and I2C hardware
 *
 * @return XST_SUCCESS on success, XST_FAILURE on failure
 *
 * Description: Initializes the NX4IO peripheral (LEDs, buttons, switches, etc.)
 *              and the I2C controller, enabling the blue channel of RGB1 LED.
 */
int do_init ( void )
{
    int status;
    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
        return XST_FAILURE;

    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
        return XST_FAILURE;

    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
        return status;

    XIic_Start ( &IicInstance );                         // Start I2C controller
    NX4IO_RGBLED_setChnlEn ( RGB1, false, false, true ); // Enable blue channel only
    return XST_SUCCESS;
}

/*--------------------------------------------------------------------------*/
/* FreeRTOS Tasks                                                           */
/*--------------------------------------------------------------------------*/
/**
 * Parse_Input_Task - Task to parse button and switch inputs
 *
 * @param p Unused parameter
 *
 * Description: Waits for GPIO interrupts via binary_sem, reads button and switch
 *              states, combines them into a 16-bit value, updates LEDs, and sends
 *              the value to the PID task via toPID queue.
 */
void Parse_Input_Task ( void* p )
{
    uint8_t  btns        = 0x00;   // Button states (bits 1-4)
    uint8_t  sws         = 0x00;   // Switch states (bits 0-7)
    uint16_t ValueToSend = 0x0000; // Combined btns (high) and sws (low)

    while ( 1 )
    {
        if ( xSemaphoreTake ( binary_sem, 500 ) ) // Wait up to 500 ticks
        {
            btns = ( NX4IO_getBtns ( ) & 0x1E );                 // Mask to bits 1-4 (BTN1-BTN4)
            sws  = (uint8_t) ( NX4IO_getSwitches ( ) & 0x00FF ); // Mask to 8 switches
            ValueToSend |= ( ( btns << 8 ) | sws );              // Combine: btns high, sws low
            NX4IO_setLEDs ( sws );                               // Reflect switch state on LEDs
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );  // Send to PID task
            ValueToSend &= 0x0000;                               // Reset for next iteration
        }
    }
}

/**
 * PID_Task - Task to manage PID control for LED brightness
 *
 * @param p Pointer to PID_t structure for LED control
 *
 * Description: Reads button/switch inputs from toPID queue, adjusts PID parameters
 *              (setpoint, Kp, Ki, Kd), computes PID output based on TSL2561 lux
 *              readings, and updates RGB LED brightness. Sends setpoint and lux
 *              to Display_Task via fromPID queue.
 */
void PID_Task ( void* p )
{
    PID_t*      pid     = (PID_t*) p;                  // PID controller instance
    float       pidOUT  = 0;                           // PID output
    uint16_t    tsl2561 = 0;                           // Lux value from TSL2561
    uint8_t     pwmLED  = 127;                         // Initial PWM duty cycle (50%)
    uint16_t    btnSws;                                // Received button/switch value
    uint32_t    setpntLux;                             // Combined setpoint and lux for display
    uint8_t     btns, sws;                             // Parsed buttons and switches
    float       baseID = 0.01, baseP = 0.1;            // Base increments for Ki/Kd and Kp
    uint8_t     baseSP        = 1, incScaling;         // Base setpoint increment and scaling
    uint8_t     btnPrint      = 0x00;                  // Toggle for printing PID info
    TickType_t  lastTick      = xTaskGetTickCount ( ); // Last update time
    static bool isInitialized = false;                 // PID initialization flag

    if ( !isInitialized )
        isInitialized = pid_init ( pid ); // Initialize PID on first run

    while ( 1 )
    {
        /* Receive button/switch data */
        if ( xQueueReceive ( toPID, &btnSws, 42 ) == pdPASS ) // Wait up to 42 ticks
        {
            btns = ( btnSws & 0x1E00 ) >> 8; // Extract buttons (bits 9-12)
            sws  = ( btnSws & 0x0FF );       // Extract switches (bits 0-7)
        }
        else
        {
            btns = 0x00; // No input received
        }

        if ( btns & 0x02 )
            btnPrint = !btnPrint; // Toggle print with BTN1

        /* Determine increment scaling based on switches 4-5 */
        if ( !( sws & 0x30 ) )
            incScaling = 1; // Both off: x1
        else if ( sws & 0x20 )
            incScaling = 10; // SW5 on: x10
        else if ( sws & 0x10 )
            incScaling = 5; // SW4 on: x5

        /* Adjust PID parameters based on switches and buttons */
        if ( sws & 0x08 ) // SW3: Adjust setpoint
        {
            if ( btns & 0x08 ) // BTN3: Increase
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, true );
            else if ( btns & 0x04 ) // BTN2: Decrease
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, false );
        }
        else // Adjust gains based on SW6-7
        {
            switch ( sws & 0xC0 )
            {
            case 0x40: // SW6: Kp
                if ( btns & 0x08 )
                    UPDATE_SATURATING ( pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, true );
                else if ( btns & 0x04 )
                    UPDATE_SATURATING ( pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, false );
                break;
            case 0x80: // SW7: Ki
                if ( btns & 0x08 )
                    UPDATE_SATURATING ( pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, true );
                else if ( btns & 0x04 )
                    UPDATE_SATURATING ( pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, false );
                break;
            case 0xC0: // SW6+SW7: Kd
                if ( btns & 0x08 )
                    UPDATE_SATURATING ( pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, true );
                else if ( btns & 0x04 )
                    UPDATE_SATURATING ( pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, false );
                break;
            }
        }

        /* Read TSL2561 channels and compute lux */
        float ch0 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_0 ); // Broadband
        float ch1 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_1 ); // Infrared
        if ( ch0 >= ch1 * 0.5 )
        {
            tsl2561 = (uint16_t) ( ch0 - ch1 * 0.5 ); // Approximate lux value
        }
        else
        {
            tsl2561 = 0; // Prevent underflow
        }
        sensor_data.luminosity = tsl2561; // Update global sensor data
        xil_printf ( "PID: L:%u\r\n", sensor_data.luminosity );
        lastTick = xTaskGetTickCount ( ); // Update timestamp

        /* Compute PID output and adjust PWM */
        pidOUT = pid_funct ( pid, tsl2561, sws );
        if ( ( pwmLED + ( pidOUT * max_duty ) ) >= max_duty )
            pwmLED = max_duty; // Saturate at max
        else if ( ( pwmLED + ( pidOUT * max_duty ) ) <= min_duty )
            pwmLED = min_duty; // Saturate at min
        else
            pwmLED = (uint8_t) ( pwmLED + ( pidOUT * max_duty ) ); // Update PWM

        NX4IO_RGBLED_setDutyCycle ( RGB1, 0, 0, pwmLED ); // Set blue channel PWM

        /* Print PID info if toggled */
        if ( btnPrint )
        {
            xil_printf ( "Setpoint Value: %d\r\n", pid->setpoint );
            xil_printf ( "Lux Value: %d\r\n", tsl2561 );
            xil_printf ( "PWM LED Duty Cycle: %d\r\n", pwmLED );
        }

        /* Send setpoint and lux to display task */
        setpntLux = ( tsl2561 << 0 ) | ( pid->setpoint << 16 ); // Lower 16: lux, Upper 16: setpoint
        xQueueSend ( fromPID, &setpntLux, mainDONT_BLOCK );

        if ( btns & 0x10 )
            print_pid ( pid ); // BTN4: Print PID gains
    }
}

/**
 * Display_Task - Task to update 7-segment display with PID data
 *
 * @param p Unused parameter
 *
 * Description: Receives setpoint and lux values from PID_Task via fromPID queue
 *              and displays them on the high (setpoint) and low (lux) 7-segment
 *              displays of the NX4IO peripheral.
 */
void Display_Task ( void* p )
{
    uint32_t recievedLux;             // Received combined setpoint and lux
    uint16_t setpnt = 0x0000, luxVal; // Parsed setpoint and lux values

    while ( 1 )
    {
        xQueueReceive ( fromPID, &recievedLux, portMAX_DELAY ); // Block until data received
        luxVal = ( recievedLux & lux_mask );                    // Extract lux (lower 16 bits)
        setpnt = ( ( recievedLux >> 16 ) & lux_mask );          // Extract setpoint (upper 16 bits)

        /* Display setpoint on high 7-segment display */
        NX410_SSEG_setAllDigits ( SSEGHI,
                                  ( setpnt / 100 ),          // Hundreds
                                  ( ( setpnt % 100 ) / 10 ), // Tens
                                  ( ( setpnt % 100 ) % 10 ), // Ones
                                  CC_BLANK,                  // Blank digit
                                  DP_NONE );                 // No decimal point

        /* Display lux on low 7-segment display */
        NX410_SSEG_setAllDigits ( SSEGLO,
                                  ( luxVal / 100 ),          // Hundreds
                                  ( ( luxVal % 100 ) / 10 ), // Tens
                                  ( ( luxVal % 100 ) % 10 ), // Ones
                                  CC_BLANK,                  // Blank digit
                                  DP_NONE );                 // No decimal point
    }
}

/*--------------------------------------------------------------------------*/
/* PID Functions                                                            */
/*--------------------------------------------------------------------------*/
/**
 * pid_init - Initialize PID controller parameters
 *
 * @param pid Pointer to PID_t structure to initialize
 * @return true on success
 *
 * Description: Sets initial PID gains (Kp, Ki, Kd), setpoint, and limits for the
 *              PID controller used to regulate LED brightness.
 */
bool pid_init ( PID_t* pid )
{
    pid->Kp         = 0.7;   // Proportional gain
    pid->Ki         = 0.2;   // Integral gain
    pid->Kd         = 0;     // Derivative gain
    pid->setpoint   = 100;   // Initial setpoint (lux)
    pid->integral   = 0;     // Integral term accumulator
    pid->prev_error = 0;     // Previous error for derivative term
    pid->delta_t    = 0.437; // Time step (seconds, approximate task period)
    pid->max_lim    = 1000;  // Maximum limit for setpoint and gains
    pid->min_lim    = 0;     // Minimum limit
    return true;
}

/**
 * pid_funct - Compute PID output for LED brightness control
 *
 * @param pid Pointer to PID_t structure
 * @param lux_value Current lux reading from TSL2561
 * @param switches Switch states to enable/disable P, I, D terms
 * @return Normalized PID output (0 to 1)
 *
 * Description: Calculates PID output using proportional, integral, and derivative
 *              terms based on error (setpoint - lux). Applies saturation to the
 *              integral term and scales output relative to setpoint.
 */
float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches )
{
    uint16_t max_int = 2048;                      // Maximum integral limit
    float    min_int = -2048;                     // Minimum integral limit
    float    error   = pid->setpoint - lux_value; // Current error

    float Pterm = ( switches & 0x01 ) ? pid->Kp * error : 0; // Proportional term (SW0)

    pid->integral += ( error * pid->delta_t ); // Update integral
    if ( pid->integral >= max_int )
        pid->integral = max_int; // Saturate integral
    else if ( pid->integral <= min_int )
        pid->integral = min_int;
    float Iterm = ( switches & 0x02 ) ? pid->Ki * pid->integral : 0; // Integral term (SW1)

    float Dterm     = ( switches & 0x04 ) ? pid->Kd * ( ( error - pid->prev_error ) / pid->delta_t )
                                          : 0; // Derivative term (SW2)
    pid->prev_error = error;                   // Update previous error

    return ( Pterm + Iterm + Dterm ) / (float) pid->setpoint; // Normalized output
}

/**
 * print_pid - Print PID gains to console
 *
 * @param pid Pointer to PID_t structure
 *
 * Description: Displays the current values of Kp, Ki, and Kd with integer and
 *              decimal parts separated for readability.
 */
void print_pid ( PID_t* pid )
{
    xil_printf ( "PID gains:\r\n" );
    xil_printf ( "Kp = %u.%02u\r\n",
                 (uint16_t) pid->Kp,
                 (uint16_t) ( ( pid->Kp - (uint16_t) pid->Kp ) * 100 ) ); // Integer.Decimal
    xil_printf ( "Ki = %u.%02u\r\n",
                 (uint16_t) pid->Ki,
                 (uint16_t) ( ( pid->Ki - (uint16_t) pid->Ki ) * 100 ) );
    xil_printf ( "Kd = %u.%02u\r\n",
                 (uint16_t) pid->Kd,
                 (uint16_t) ( ( pid->Kd - (uint16_t) pid->Kd ) * 100 ) );
}

/*--------------------------------------------------------------------------*/
/* Font Data                                                                */
/*--------------------------------------------------------------------------*/
/**
 * @brief Font data for 8x16 pixel characters (ASCII 32-126) for OLED display.
 *        Each character is 16 bytes (8x16 bits), with 4-byte header.
 */
const uint8_t ssd1306xled_font8x16[] = {
    0x00, 0x08, 0x10, 0x20, // Font header (width, height, etc.)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33,
    0x30, 0x00, 0x00, 0x00, // !
    0x00, 0x10, 0x0C, 0x06, 0x10, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // "
    0x40, 0xC0, 0x78, 0x40, 0xC0, 0x78, 0x40, 0x00, 0x04, 0x3F, 0x04, 0x04,
    0x3F, 0x04, 0x04, 0x00, // #
    0x00, 0x70, 0x88, 0xFC, 0x08, 0x30, 0x00, 0x00, 0x00, 0x18, 0x20, 0xFF,
    0x21, 0x1E, 0x00, 0x00, // $
    0xF0, 0x08, 0xF0, 0x00, 0xE0, 0x18, 0x00, 0x00, 0x00, 0x21, 0x1C, 0x03,
    0x1E, 0x21, 0x1E, 0x00, // %
    0x00, 0xF0, 0x08, 0x88, 0x70, 0x00, 0x00, 0x00, 0x1E, 0x21, 0x23, 0x24,
    0x19, 0x27, 0x21, 0x10, // &
    0x10, 0x16, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // '
    0x00, 0x00, 0x00, 0xE0, 0x18, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x07,
    0x18, 0x20, 0x40, 0x00, // (
    0x00, 0x02, 0x04, 0x18, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x18,
    0x07, 0x00, 0x00, 0x00, // )
    0x40, 0x40, 0x80, 0xF0, 0x80, 0x40, 0x40, 0x00, 0x02, 0x02, 0x01, 0x0F,
    0x01, 0x02, 0x02, 0x00, // *
    0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x1F,
    0x01, 0x01, 0x01, 0x00, // +
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xB0, 0x70, 0x00,
    0x00, 0x00, 0x00, 0x00, // ,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, // -
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00,
    0x00, 0x00, 0x00, 0x00, // .
    0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x18, 0x04, 0x00, 0x60, 0x18, 0x06,
    0x01, 0x00, 0x00, 0x00, // /
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x0F, 0x10, 0x20,
    0x20, 0x10, 0x0F, 0x00, // 0
    0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F,
    0x20, 0x20, 0x00, 0x00, // 1
    0x00, 0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x30, 0x28, 0x24,
    0x22, 0x21, 0x30, 0x00, // 2
    0x00, 0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00, 0x00, 0x18, 0x20, 0x20,
    0x20, 0x11, 0x0E, 0x00, // 3
    0x00, 0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x07, 0x04, 0x24,
    0x24, 0x3F, 0x24, 0x00, // 4
    0x00, 0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00, 0x19, 0x21, 0x20,
    0x20, 0x11, 0x0E, 0x00, // 5
    0x00, 0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00, 0x00, 0x0F, 0x11, 0x20,
    0x20, 0x11, 0x0E, 0x00, // 6
    0x00, 0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00, 0x00, 0x3F,
    0x00, 0x00, 0x00, 0x00, // 7
    0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x1C, 0x22, 0x21,
    0x21, 0x22, 0x1C, 0x00, // 8
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x00, 0x31, 0x22,
    0x22, 0x11, 0x0F, 0x00, // 9
    0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x30, 0x00, 0x00, 0x00, // :
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60,
    0x00, 0x00, 0x00, 0x00, // ;
    0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x01, 0x02, 0x04,
    0x08, 0x10, 0x20, 0x00, // <
    0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x00, // =
    0x00, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x20, 0x10, 0x08,
    0x04, 0x02, 0x01, 0x00, // >
    0x00, 0x70, 0x48, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x36, 0x01, 0x00, 0x00, // ?
    0xC0, 0x30, 0xC8, 0x28, 0xE8, 0x10, 0xE0, 0x00, 0x07, 0x18, 0x27, 0x24,
    0x23, 0x14, 0x0B, 0x00, // @
    0x00, 0x00, 0xC0, 0x38, 0xE0, 0x00, 0x00, 0x00, 0x20, 0x3C, 0x23, 0x02,
    0x02, 0x27, 0x38, 0x20, // A
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x20,
    0x20, 0x11, 0x0E, 0x00, // B
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00, 0x07, 0x18, 0x20, 0x20,
    0x20, 0x10, 0x08, 0x00, // C
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x20, 0x3F, 0x20, 0x20,
    0x20, 0x10, 0x0F, 0x00, // D
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F, 0x20, 0x20,
    0x23, 0x20, 0x18, 0x00, // E
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F, 0x20, 0x00,
    0x03, 0x00, 0x00, 0x00, // F
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x07, 0x18, 0x20, 0x20,
    0x22, 0x1E, 0x02, 0x00, // G
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F, 0x21, 0x01,
    0x01, 0x21, 0x3F, 0x20, // H
    0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F,
    0x20, 0x20, 0x00, 0x00, // I
    0x00, 0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0xC0, 0x80, 0x80, 0x80,
    0x7F, 0x00, 0x00, 0x00, // J
    0x08, 0xF8, 0x88, 0xC0, 0x28, 0x18, 0x08, 0x00, 0x20, 0x3F, 0x20, 0x01,
    0x26, 0x38, 0x20, 0x00, // K
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x20,
    0x20, 0x20, 0x30, 0x00, // L
    0x08, 0xF8, 0xF8, 0x00, 0xF8, 0xF8, 0x08, 0x00, 0x20, 0x3F, 0x00, 0x3F,
    0x00, 0x3F, 0x20, 0x00, // M
    0x08, 0xF8, 0x30, 0xC0, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F, 0x20, 0x00,
    0x07, 0x18, 0x3F, 0x00, // N
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x10, 0x20, 0x20,
    0x20, 0x10, 0x0F, 0x00, // O
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x20, 0x3F, 0x21, 0x01,
    0x01, 0x01, 0x00, 0x00, // P
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x18, 0x24, 0x24,
    0x38, 0x50, 0x4F, 0x00, // Q
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x20, 0x3F, 0x20, 0x00,
    0x03, 0x0C, 0x30, 0x20, // R
    0x00, 0x70, 0x88, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x38, 0x20, 0x21,
    0x21, 0x22, 0x1C, 0x00, // S
    0x18, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x18, 0x00, 0x00, 0x00, 0x20, 0x3F,
    0x20, 0x00, 0x00, 0x00, // T
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x00, 0x1F, 0x20, 0x20,
    0x20, 0x20, 0x1F, 0x00, // U
    0x08, 0x78, 0x88, 0x00, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x07, 0x38,
    0x0E, 0x01, 0x00, 0x00, // V
    0xF8, 0x08, 0x00, 0xF8, 0x00, 0x08, 0xF8, 0x00, 0x03, 0x3C, 0x07, 0x00,
    0x07, 0x3C, 0x03, 0x00, // W
    0x08, 0x18, 0x68, 0x80, 0x80, 0x68, 0x18, 0x08, 0x20, 0x30, 0x2C, 0x03,
    0x03, 0x2C, 0x30, 0x20, // X
    0x08, 0x38, 0xC8, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00, 0x20, 0x3F,
    0x20, 0x00, 0x00, 0x00, // Y
    0x10, 0x08, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x20, 0x38, 0x26, 0x21,
    0x20, 0x20, 0x18, 0x00, // Z
    0x00, 0x00, 0x00, 0xFE, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x7F,
    0x40, 0x40, 0x40, 0x00, // [
    0x00, 0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40,
    0x7F, 0x00, 0x00, 0x00, // ]
    0x00, 0x00, 0x04, 0x02, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // ^
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x80, 0x80, // _
    0x00, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // `
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x19, 0x24, 0x22,
    0x22, 0x22, 0x3F, 0x20, // a
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x11, 0x20,
    0x20, 0x11, 0x0E, 0x00, // b
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x0E, 0x11, 0x20,
    0x20, 0x20, 0x11, 0x00, // c
    0x00, 0x00, 0x00, 0x80, 0x80, 0x88, 0xF8, 0x00, 0x00, 0x0E, 0x11, 0x20,
    0x20, 0x10, 0x3F, 0x20, // d
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x22, 0x22,
    0x22, 0x22, 0x13, 0x00, // e
    0x00, 0x80, 0x80, 0xF0, 0x88, 0x88, 0x88, 0x18, 0x00, 0x20, 0x20, 0x3F,
    0x20, 0x20, 0x00, 0x00, // f
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x6B, 0x94, 0x94,
    0x94, 0x93, 0x60, 0x00, // g
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F, 0x21, 0x00,
    0x00, 0x20, 0x3F, 0x20, // h
    0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F,
    0x20, 0x20, 0x00, 0x00, // i
    0x00, 0x00, 0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0xC0, 0x80, 0x80,
    0x80, 0x7F, 0x00, 0x00, // j
    0x08, 0xF8, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F, 0x24, 0x02,
    0x2D, 0x30, 0x20, 0x00, // k
    0x00, 0x08, 0x08, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F,
    0x20, 0x20, 0x00, 0x00, // l
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F, 0x20, 0x00,
    0x3F, 0x20, 0x00, 0x3F, // m
    0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F, 0x21, 0x00,
    0x00, 0x20, 0x3F, 0x20, // n
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x20, 0x20,
    0x20, 0x20, 0x1F, 0x00, // o
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xA1, 0x20,
    0x20, 0x11, 0x0E, 0x00, // p
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x0E, 0x11, 0x20,
    0x20, 0xA0, 0xFF, 0x80, // q
    0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x20, 0x3F, 0x21,
    0x20, 0x00, 0x01, 0x00, // r
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x33, 0x24, 0x24,
    0x24, 0x24, 0x19, 0x00, // s
    0x00, 0x80, 0x80, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
    0x20, 0x20, 0x00, 0x00, // t
    0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x1F, 0x20, 0x20,
    0x20, 0x10, 0x3F, 0x20, // u
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x01, 0x0E, 0x30,
    0x08, 0x06, 0x01, 0x00, // v
    0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x80, 0x0F, 0x30, 0x0C, 0x03,
    0x0C, 0x30, 0x0F, 0x00, // w
    0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x31, 0x2E,
    0x0E, 0x31, 0x20, 0x00, // x
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x81, 0x8E, 0x70,
    0x18, 0x06, 0x01, 0x00, // y
    0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x21, 0x30, 0x2C,
    0x22, 0x21, 0x30, 0x00, // z
    0x00, 0x00, 0x00, 0x00, 0x80, 0x7C, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3F, 0x40, 0x40, // {
    0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0x00, 0x00, 0x00, // |
    0x00, 0x02, 0x02, 0x7C, 0x80, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x3F,
    0x00, 0x00, 0x00, 0x00, // }
    0x00, 0x06, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // ~
};