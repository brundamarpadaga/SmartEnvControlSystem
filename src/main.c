/*
 * Summary:
 * This is a FreeRTOS-based embedded application for environmental monitoring and control.
 * It interfaces with a TSL2561 light sensor, BME280 temperature/humidity sensor, and OLED display.
 * The system uses PID controllers to maintain environmental parameters, with user input via buttons
 * and switches. Visual feedback is provided through RGB LEDs and a 7-segment display.
 * Multiple tasks handle sensor reading, control, input parsing, and display updates, synchronized
 * using semaphores and queues. The system runs on a Microblaze processor with AXI GPIO and I2C
 * peripherals.
 */

#include "main.h"

#include <stdlib.h>

// Global variable declarations
XGpio                    xInputGPIOInstance;   // GPIO instance for input handling
XIic                     IicInstance;          // I2C instance for sensor communication
struct bme280_calib_data calib_data;           // BME280 sensor calibration data
SemaphoreHandle_t        binary_sem;           // Semaphore for GPIO interrupt synchronization
SemaphoreHandle_t        bme280_sem;           // Semaphore for BME280 sensor access
SemaphoreHandle_t        oled_sem;             // Semaphore for OLED display access
SemaphoreHandle_t        i2c_sem;              // Semaphore for I2C bus access
xQueueHandle             toPID       = NULL;   // Queue for sending data to PID task
xQueueHandle             fromPID     = NULL;   // Queue for receiving data from PID task
SensorData_t             sensor_data = { 0 };  // Structure to hold sensor readings

/*
 * Function: vAssertCalled
 * Description: FreeRTOS assertion handler that catches runtime errors.
 *              Prints the file and line number where assertion failed,
 *              disables interrupts, and enters an infinite loop to halt execution.
 * Parameters:
 *   - file: Pointer to the source file name where assertion occurred
 *   - line: Line number in the source file
 * Returns: None (never returns)
 */
void vAssertCalled ( const char* file, unsigned long line )
{
    xil_printf ( "[ERROR] FreeRTOS assertion failed in %s, line %lu\r\n",
                 file,
                 line );         // Print error details
    taskDISABLE_INTERRUPTS ( );  // Prevent further interrupt handling
    while ( 1 );                 // Infinite loop to halt system
}

/*
 * Function: main
 * Description: Main entry point of the application. Initializes hardware,
 *              sets up FreeRTOS synchronization primitives, creates tasks,
 *              and starts the scheduler. Handles initial sensor and display setup.
 * Parameters: None
 * Returns: int (never returns under normal operation; -1 on error)
 */
int main ( void )
{
    xil_printf ( "Hello from FreeRTOS LUX, BME280, and OLED Controller\r\n" );  // Startup message

    prvSetupHardware ( );  // Configure GPIO and interrupt system

    if ( do_init ( ) != XST_SUCCESS )  // Initialize hardware peripherals
    {
        xil_printf ( "[ERROR] Hardware initialization failed\r\n" );
        return -1;
    }

    // Create I2C semaphore for bus access control
    i2c_sem = xSemaphoreCreateBinary ( );
    if ( i2c_sem == NULL )
    {
        xil_printf ( "[ERROR] I2C semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( i2c_sem );  // Release semaphore initially

    // Initialize TSL2561 light sensor via I2C
    if ( tsl2561_init ( &IicInstance ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] TSL2561 initialization failed\r\n" );
        return -1;
    }

    // Initialize BME280 sensor and read its calibration data
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

    // Initialize OLED display
    if ( lcd_init ( &IicInstance ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] OLED initialization failed\r\n" );
        return -1;
    }

    vSemaphoreCreateBinary ( binary_sem );  // Create semaphore for GPIO interrupts

    // Create semaphore for BME280 access synchronization
    bme280_sem = xSemaphoreCreateBinary ( );
    if ( bme280_sem == NULL )
    {
        xil_printf ( "[ERROR] BME280 semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( bme280_sem );  // Initially available

    // Create semaphore for OLED access synchronization
    oled_sem = xSemaphoreCreateBinary ( );
    if ( oled_sem == NULL )
    {
        xil_printf ( "[ERROR] OLED semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( oled_sem );  // Initially available

    // Create communication queues between tasks
    toPID   = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );  // Input to PID task
    fromPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint32_t ) );  // Output from PID task
    configASSERT ( toPID );                                            // Verify queue creation
    configASSERT ( fromPID );

    // Create all system tasks with appropriate stack sizes and priorities
    xTaskCreate ( Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL );
    xTaskCreate ( PID_Task, "PID", configMINIMAL_STACK_SIZE * 2, &sensor_data, 1, NULL );
    xTaskCreate ( Display_Task, "Disp", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL );
    xTaskCreate ( BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL );
    xTaskCreate ( LCD_Task, "LCD", configMINIMAL_STACK_SIZE * 4, &sensor_data, 2, NULL );

    xil_printf ( "Starting the scheduler\r\n" );
    vTaskStartScheduler ( );  // Launch FreeRTOS scheduler

    return -1;  // Only reached if scheduler fails to start
}

/*
 * Function: prvSetupHardware
 * Description: Configures the GPIO hardware and sets up interrupt handling
 *              for button inputs. Initializes GPIO pins as inputs and enables
 *              interrupts for user interaction.
 * Parameters: None
 * Returns: None
 */
void prvSetupHardware ( void )
{
    uint32_t            xStatus;
    const unsigned char ucSetToInput = 0xFFU;  // All pins as inputs

    xil_printf ( "Initializing GPIO's\r\n" );

    xStatus = XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );  // Init GPIO
    if ( xStatus == XST_SUCCESS )
    {
        // Install interrupt handler
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL );
        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );
            XGpio_SetDataDirection (
                &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );  // Buttons as inputs
            XGpio_SetDataDirection (
                &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );  // Switches as inputs
            vPortEnableInterrupt (
                XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR );  // Enable interrupt line
            XGpio_InterruptEnable ( &xInputGPIOInstance,
                                    XGPIO_IR_CH1_MASK );          // Enable channel 1 interrupts
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );  // Enable GPIO interrupts globally
        }
    }
    configASSERT ( ( xStatus == pdPASS ) );  // Assert successful setup
}

/*
 * Function: gpio_intr
 * Description: Interrupt service routine for GPIO events. Signals the input
 *              parsing task via semaphore and clears the interrupt flag.
 * Parameters:
 *   - pvUnused: Unused parameter (required by interrupt handler signature)
 * Returns: None
 */
void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Flag for context switch

    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );  // Signal input task
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );      // Clear interrupt
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );                  // Yield if necessary
}

/*
 * Function: do_init
 * Description: Initializes peripheral hardware including NX4IO and I2C controller.
 *              Enables RGB LED channels for visual feedback.
 * Parameters: None
 * Returns: int (XST_SUCCESS on success, XST_FAILURE on failure)
 */
int do_init ( void )
{
    int status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] NX4IO init failed\r\n" );
        return XST_FAILURE;
    }

    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        xil_printf ( "[ERROR] I2C config lookup failed\r\n" );
        return XST_FAILURE;
    }

    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C init failed: %d\r\n", status );
        return status;
    }

    status = XIic_Start ( &IicInstance );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C start failed: %d\r\n", status );
        return status;
    }

    NX4IO_RGBLED_setChnlEn ( RGB1, true, true, true );
    NX4IO_RGBLED_setChnlEn ( RGB2, true, true, true );
    return XST_SUCCESS;
}
/*
 * Function: Parse_Input_Task
 * Description: FreeRTOS task that monitors button and switch inputs. Responds
 *              to interrupts and periodically polls switches, sending combined
 *              input data to the PID task via queue.
 * Parameters:
 *   - p: Unused task parameter
 * Returns: None (infinite loop)
 */
void Parse_Input_Task ( void* p )
{
    uint8_t  btns        = 0x00;    // Current button states
    uint8_t  sws         = 0x00;    // Current switch states
    uint16_t ValueToSend = 0x0000;  // Data to send to PID task

    while ( 1 )
    {
        if ( xSemaphoreTake ( binary_sem, 500 ) )  // Wait for interrupt or timeout
        {
            ValueToSend &= 0x0000;                                // Clear previous value
            btns = ( NX4IO_getBtns ( ) & 0x1F );                  // Read 5 button bits
            sws  = (uint8_t) ( NX4IO_getSwitches ( ) & 0x00FF );  // Read 8 switch bits
            ValueToSend |= ( ( btns << 8 ) | sws );               // Combine into 16-bit value
            NX4IO_setLEDs ( sws );                                // Update LEDs with switch state
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );   // Send to PID task
        }
        else  // Timeout case - periodic polling
        {
            sws = (uint8_t) ( NX4IO_getSwitches ( ) & 0x00FF );  // Read switches
            if ( ( sws & ValueToSend ) != sws )                  // Check for switch change
            {
                ValueToSend &= 0x0000;                               // Clear previous value
                ValueToSend |= sws;                                  // Update with new switch state
                NX4IO_setLEDs ( sws );                               // Reflect on LEDs
                xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );  // Send update
            }
            vTaskDelay ( 500 );  // Wait 500 ticks before next poll
        }
    }
}

/*
 * Function: correctedSignal
 * Description: Applies PID output to an environmental control signal (PWM),
 *              ensuring it stays within defined duty cycle limits.
 * Parameters:
 *   - enviro: Current signal value
 *   - pidOut: PID controller output
 * Returns: uint8_t (corrected signal value)
 */
uint8_t correctedSignal ( uint8_t enviro, float pidOut )
{
    if ( ( enviro + ( pidOut * max_duty ) ) >= max_duty )  // Check upper limit
    {
        enviro = max_duty;  // Cap at maximum
    }
    else if ( ( enviro + ( pidOut * max_duty ) ) <= min_duty )  // Check lower limit
    {
        enviro = min_duty;  // Floor at minimum
    }
    else  // Within bounds
    {
        enviro = (uint8_t) ( enviro + ( pidOut * max_duty ) );  // Apply correction
    }
    return enviro;
}

/*
 * Function: displayHelper
 * Description: Updates setpoint based on button inputs and sends sensor value
 *              and setpoint to display task via queue.
 * Parameters:
 *   - pid: Pointer to PID controller structure
 *   - btns: Current button states
 *   - sensorVal: Current sensor reading
 *   - incr: Increment value for setpoint adjustment
 * Returns: None
 */
void displayHelper ( PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr )
{
    uint32_t message = 0x00000000;  // Message to send

    if ( btns & 0x08 )
        pid->setpoint += incr;  // Button 3: increment setpoint
    else if ( btns & 0x04 )
        pid->setpoint -= incr;  // Button 2: decrement setpoint

    message |= ( ( sensorVal << 0 ) | ( pid->setpoint << 16 ) );  // Pack values
    xQueueSend ( fromPID, &message, mainDONT_BLOCK );             // Send to display
}

/*
 * Function: PID_Task
 * Description: FreeRTOS task implementing PID control for light, temperature,
 *              and humidity. Updates PWM signals and LED indicators based on
 *              sensor readings and user inputs.
 * Parameters:
 *   - p: Pointer to SensorData_t structure
 * Returns: None (infinite loop)
 */
void PID_Task ( void* p )
{
    SensorData_t* sensor_data = (SensorData_t*) p;              // Sensor data access
    float         luxOUT = 0, tempOUT = 0, humOUT = 0;          // PID outputs
    uint16_t      tsl2561  = 0;                                 // Light sensor value
    uint8_t       envLight = 127, envTemp = 127, envHum = 127;  // PWM values
    uint16_t      btnSws;                                       // Input data
    uint8_t       btns        = 0x10, sws;                      // Current states
    TickType_t    lastLuxTick = xTaskGetTickCount ( );          // Last light update
    TickType_t    lastBMETick = xTaskGetTickCount ( );          // Last BME update

    static bool  isInitialized = false;    // PID init flag
    static PID_t pidLux, pidTemp, pidHum;  // PID controllers

    if ( !isInitialized )  // One-time PID initialization
    {
        isInitialized = ( pid_init ( &pidLux ) && pid_init ( &pidTemp ) && pid_init ( &pidHum ) );
    }

    while ( 1 )
    {
        // Check for new input data
        if ( xQueueReceive ( toPID, &btnSws, mainDONT_BLOCK ) == pdPASS )
        {
            if ( btnSws & 0x1300 )  // Special case handling
                btns = ( btnSws & 0x1F00 ) >> 8;
            else
                btns = ( ( btnSws & 0x1F00 ) >> 8 ) | 0x13;
            sws = ( btnSws & 0x0FF );  // Extract switch bits
        }
        else
        {
            btns &= 0x13;  // Maintain persistent bits
        }

        // Light sensor control (~42ms interval)
        TickType_t currentLuxTick = xTaskGetTickCount ( );
        if ( currentLuxTick - lastLuxTick >= 42 )
        {
            float ch0 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_0 );  // Read IR+visible
            float ch1 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_1 );  // Read IR
            tsl2561   = ( ch0 >= ch1 * 0.5 ) ? (uint16_t) ( ch0 - ch1 * 0.5 )
                                             : 0;  // Calculate visible light
            sensor_data->luminosity = tsl2561;     // Store reading
            pidLux.delta_t =
                ( ( currentLuxTick - lastLuxTick ) * ( 1 / 100.0f ) );  // Update time delta
            lastLuxTick = currentLuxTick;                               // Update timestamp
            luxOUT      = pid_funct ( &pidLux, tsl2561, sws );          // Calculate PID
        }

        // BME280 control
        TickType_t currentBMETick = xTaskGetTickCount ( );
        if ( xSemaphoreTake ( bme280_sem, mainDONT_BLOCK ) )  // Access BME280 data
        {
            pidHum.delta_t =
                ( ( currentBMETick - lastBMETick ) * ( 1 / 100.0f ) );  // Humidity delta
            pidTemp.delta_t = ( ( currentBMETick - lastBMETick ) * ( 1 / 100.0f ) );  // Temp delta
            lastBMETick     = currentBMETick;  // Update timestamp
            humOUT          = pid_funct (
                &pidHum, (uint16_t) ( sensor_data->humidity / 1024 ), sws );  // Humidity PID
            tempOUT = pid_funct (
                &pidTemp, (uint16_t) ( sensor_data->temperature / 100 ), sws );  // Temp PID
            xSemaphoreGive ( bme280_sem );  // Release semaphore
        }

        // Update display based on selected parameter
        switch ( btns & 0x13 )
        {
            case 0x10:
                displayHelper ( &pidHum, btns, sensor_data->humidity / 1024, 1 );
                break;  // Humidity
            case 0x02:
                displayHelper ( &pidTemp, btns, sensor_data->temperature / 100, 1 );
                break;  // Temperature
            case 0x01:
                displayHelper ( &pidLux, btns, tsl2561, 1 );
                break;  // Lux
        }

        // Apply PID corrections
        envLight = correctedSignal ( envLight, luxOUT );  // Light PWM
        envHum   = correctedSignal ( envHum, humOUT );    // Humidity PWM
        envTemp  = correctedSignal ( envTemp, tempOUT );  // Temperature PWM

        // RGB1 status indicator
        if ( ( ( humOUT >= -0.05 ) && ( humOUT < 0.05 ) ) &&
             ( ( tempOUT >= -0.05 ) && ( tempOUT < 0.05 ) ) )
            NX4IO_RGBLED_setDutyCycle ( RGB1, min_duty, max_duty, min_duty );  // Green: both stable
        else if ( !( ( humOUT >= -0.05 ) && ( humOUT < 0.05 ) ) &&
                  !( ( tempOUT >= -0.05 ) && ( tempOUT < 0.05 ) ) )
            NX4IO_RGBLED_setDutyCycle (
                RGB1, max_duty, min_duty, max_duty );  // Purple: both unstable
        else if ( !( ( humOUT >= -0.05 ) && ( humOUT < 0.05 ) ) )
            NX4IO_RGBLED_setDutyCycle (
                RGB1, min_duty, min_duty, max_duty );  // Blue: humidity unstable
        else
            NX4IO_RGBLED_setDutyCycle ( RGB1, max_duty, min_duty, min_duty );  // Red: temp unstable

        // RGB2 control indicator
        switch ( sws & 0x07 )
        {
            case 0x01:
                NX4IO_RGBLED_setDutyCycle (
                    RGB2, envTemp, (uint8_t) ( 0.25 * max_duty ), envLight );
                break;
            case 0x02:
                NX4IO_RGBLED_setDutyCycle (
                    RGB2, envTemp, (uint8_t) ( 0.50 * max_duty ), envLight );
                break;
            case 0x04:
                NX4IO_RGBLED_setDutyCycle (
                    RGB2, envTemp, (uint8_t) ( 0.75 * max_duty ), envLight );
                break;
            default:
                NX4IO_RGBLED_setDutyCycle ( RGB2, envTemp, ( envHum + envTemp ) / 2, envLight );
                break;
        }
    }
}

/*
 * Function: Display_Task
 * Description: FreeRTOS task that updates the 7-segment display with setpoint
 *              and current values received from the PID task.
 * Parameters:
 *   - p: Unused task parameter
 * Returns: None (infinite loop)
 */
void Display_Task ( void* p )
{
    uint32_t receivedLux;           // Received data
    uint16_t setpnt = 0x0000, val;  // Setpoint and value

    while ( 1 )
    {
        xQueueReceive ( fromPID, &receivedLux, portMAX_DELAY );  // Wait for data
        val    = ( receivedLux & lux_mask );                     // Extract current value
        setpnt = ( ( receivedLux >> 16 ) & lux_mask );           // Extract setpoint

        // Update upper display with setpoint
        NX410_SSEG_setAllDigits ( SSEGHI,
                                  ( setpnt / 100 ),           // Hundreds digit
                                  ( ( setpnt % 100 ) / 10 ),  // Tens digit
                                  ( ( setpnt % 100 ) % 10 ),  // Ones digit
                                  CC_BLANK,                   // Blank digit
                                  DP_NONE );                  // No decimal point
        // Update lower display with current value
        NX410_SSEG_setAllDigits ( SSEGLO,
                                  ( val / 100 ),
                                  ( ( val % 100 ) / 10 ),
                                  ( ( val % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
    }
}

/*
 * Function: pid_init
 * Description: Initializes a PID controller with default parameters.
 * Parameters:
 *   - pid: Pointer to PID_t structure to initialize
 * Returns: bool (true on success)
 */
bool pid_init ( PID_t* pid )
{
    pid->Kp         = 0.7;   // Set proportional gain
    pid->Ki         = 0.2;   // Set integral gain
    pid->Kd         = 0.01;  // Set derivative gain
    pid->setpoint   = 100;   // Set default setpoint
    pid->integral   = 0;     // Clear integral accumulator
    pid->prev_error = 0;     // Clear previous error
    pid->delta_t    = 1;     // Set default time step
    pid->max_lim    = 1000;  // Set maximum output limit
    pid->min_lim    = 0;     // Set minimum output limit
    return true;
}

/*
 * Function: pid_funct
 * Description: Implements PID control algorithm with configurable P, I, D terms
 *              based on switch settings. Normalizes output to setpoint.
 * Parameters:
 *   - pid: Pointer to PID_t structure
 *   - value: Current sensor value
 *   - switches: Switch states controlling PID terms
 * Returns: float (PID control output)
 */
float pid_funct ( PID_t* pid, uint16_t value, uint8_t switches )
{
    uint16_t max_int = 1024;                   // Integral upper limit
    float    min_int = -1024;                  // Integral lower limit
    float    error   = pid->setpoint - value;  // Calculate error

    float Pterm = ( switches & 0x01 ) ? pid->Kp * error : 0;  // Proportional term (SW0)
    pid->integral += ( error * pid->delta_t );                // Update integral
    if ( pid->integral >= max_int )
        pid->integral = max_int;  // Clamp upper bound
    else if ( pid->integral <= min_int )
        pid->integral = min_int;                                          // Clamp lower bound
    float Iterm     = ( switches & 0x02 ) ? pid->Ki * pid->integral : 0;  // Integral term (SW1)
    float Dterm     = ( switches & 0x04 ) ? pid->Kd * ( ( error - pid->prev_error ) / pid->delta_t )
                                          : 0;  // Derivative (SW2)
    pid->prev_error = error;                    // Store error for next iteration

    return ( Pterm + Iterm + Dterm ) / (float) pid->setpoint;  // Return normalized output
}

/* Font data (unchanged) */
const uint8_t ssd1306xled_font8x16[] = {
    0x00, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
    0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x33, 0x30, 0x00, 0x00, 0x00,  // !
    0x00, 0x10, 0x0C, 0x06, 0x10, 0x0C, 0x06, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // "
    0x40, 0xC0, 0x78, 0x40, 0xC0, 0x78, 0x40, 0x00, 0x04, 0x3F,
    0x04, 0x04, 0x3F, 0x04, 0x04, 0x00,  // #
    0x00, 0x70, 0x88, 0xFC, 0x08, 0x30, 0x00, 0x00, 0x00, 0x18,
    0x20, 0xFF, 0x21, 0x1E, 0x00, 0x00,  // $
    0xF0, 0x08, 0xF0, 0x00, 0xE0, 0x18, 0x00, 0x00, 0x00, 0x21,
    0x1C, 0x03, 0x1E, 0x21, 0x1E, 0x00,  // %
    0x00, 0xF0, 0x08, 0x88, 0x70, 0x00, 0x00, 0x00, 0x1E, 0x21,
    0x23, 0x24, 0x19, 0x27, 0x21, 0x10,  // &
    0x10, 0x16, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // '
    0x00, 0x00, 0x00, 0xE0, 0x18, 0x04, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x07, 0x18, 0x20, 0x40, 0x00,  // (
    0x00, 0x02, 0x04, 0x18, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x20, 0x18, 0x07, 0x00, 0x00, 0x00,  // )
    0x40, 0x40, 0x80, 0xF0, 0x80, 0x40, 0x40, 0x00, 0x02, 0x02,
    0x01, 0x0F, 0x01, 0x02, 0x02, 0x00,  // *
    0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x01, 0x1F, 0x01, 0x01, 0x01, 0x00,  // +
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xB0,
    0x70, 0x00, 0x00, 0x00, 0x00, 0x00,  // ,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01,  // -
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00,  // .
    0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x18, 0x04, 0x00, 0x60,
    0x18, 0x06, 0x01, 0x00, 0x00, 0x00,  // /
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x0F,
    0x10, 0x20, 0x20, 0x10, 0x0F, 0x00,  // 0
    0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
    0x20, 0x3F, 0x20, 0x20, 0x00, 0x00,  // 1
    0x00, 0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x30,
    0x28, 0x24, 0x22, 0x21, 0x30, 0x00,  // 2
    0x00, 0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00, 0x00, 0x18,
    0x20, 0x20, 0x20, 0x11, 0x0E, 0x00,  // 3
    0x00, 0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x07,
    0x04, 0x24, 0x24, 0x3F, 0x24, 0x00,  // 4
    0x00, 0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00, 0x19,
    0x21, 0x20, 0x20, 0x11, 0x0E, 0x00,  // 5
    0x00, 0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00, 0x00, 0x0F,
    0x11, 0x20, 0x20, 0x11, 0x0E, 0x00,  // 6
    0x00, 0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x00,  // 7
    0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x1C,
    0x22, 0x21, 0x21, 0x22, 0x1C, 0x00,  // 8
    0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x00,
    0x31, 0x22, 0x22, 0x11, 0x0F, 0x00,  // 9
    0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x30, 0x00, 0x00, 0x00,  // :
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x60, 0x00, 0x00, 0x00, 0x00,  // ;
    0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x01,
    0x02, 0x04, 0x08, 0x10, 0x20, 0x00,  // <
    0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x00,  // =
    0x00, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x20,
    0x10, 0x08, 0x04, 0x02, 0x01, 0x00,  // >
    0x00, 0x70, 0x48, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x36, 0x01, 0x00, 0x00,  // ?
    0xC0, 0x30, 0xC8, 0x28, 0xE8, 0x10, 0xE0, 0x00, 0x07, 0x18,
    0x27, 0x24, 0x23, 0x14, 0x0B, 0x00,  // @
    0x00, 0x00, 0xC0, 0x38, 0xE0, 0x00, 0x00, 0x00, 0x20, 0x3C,
    0x23, 0x02, 0x02, 0x27, 0x38, 0x20,  // A
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00, 0x20, 0x3F,
    0x20, 0x20, 0x20, 0x11, 0x0E, 0x00,  // B
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00, 0x07, 0x18,
    0x20, 0x20, 0x20, 0x10, 0x08, 0x00,  // C
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x20, 0x3F,
    0x20, 0x20, 0x20, 0x10, 0x0F, 0x00,  // D
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F,
    0x20, 0x20, 0x23, 0x20, 0x18, 0x00,  // E
    0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F,
    0x20, 0x00, 0x03, 0x00, 0x00, 0x00,  // F
    0xC0, 0x30, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x07, 0x18,
    0x20, 0x20, 0x22, 0x1E, 0x02, 0x00,  // G
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F,
    0x21, 0x01, 0x01, 0x21, 0x3F, 0x20,  // H
    0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0x00, 0x00, 0x20,
    0x20, 0x3F, 0x20, 0x20, 0x00, 0x00,  // I
    0x00, 0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0xC0, 0x80,
    0x80, 0x80, 0x7F, 0x00, 0x00, 0x00,  // J
    0x08, 0xF8, 0x88, 0xC0, 0x28, 0x18, 0x08, 0x00, 0x20, 0x3F,
    0x20, 0x01, 0x26, 0x38, 0x20, 0x00,  // K
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3F,
    0x20, 0x20, 0x20, 0x20, 0x30, 0x00,  // L
    0x08, 0xF8, 0xF8, 0x00, 0xF8, 0xF8, 0x08, 0x00, 0x20, 0x3F,
    0x00, 0x3F, 0x00, 0x3F, 0x20, 0x00,  // M
    0x08, 0xF8, 0x30, 0xC0, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F,
    0x20, 0x00, 0x07, 0x18, 0x3F, 0x00,  // N
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x10,
    0x20, 0x20, 0x20, 0x10, 0x0F, 0x00,  // O
    0x08, 0xF8, 0x08, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x20, 0x3F,
    0x21, 0x01, 0x01, 0x01, 0x00, 0x00,  // P
    0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x18,
    0x24, 0x24, 0x38, 0x50, 0x4F, 0x00,  // Q
    0x08, 0xF8, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x20, 0x3F,
    0x20, 0x00, 0x03, 0x0C, 0x30, 0x20,  // R
    0x00, 0x70, 0x88, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x38,
    0x20, 0x21, 0x21, 0x22, 0x1C, 0x00,  // S
    0x18, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x18, 0x00, 0x00, 0x00,
    0x20, 0x3F, 0x20, 0x00, 0x00, 0x00,  // T
    0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x00, 0x1F,
    0x20, 0x20, 0x20, 0x20, 0x1F, 0x00,  // U
    0x08, 0x78, 0x88, 0x00, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00,
    0x07, 0x38, 0x0E, 0x01, 0x00, 0x00,  // V
    0xF8, 0x08, 0x00, 0xF8, 0x00, 0x08, 0xF8, 0x00, 0x03, 0x3C,
    0x07, 0x00, 0x07, 0x3C, 0x03, 0x00,  // W
    0x08, 0x18, 0x68, 0x80, 0x80, 0x68, 0x18, 0x08, 0x20, 0x30,
    0x2C, 0x03, 0x03, 0x2C, 0x30, 0x20,  // X
    0x08, 0x38, 0xC8, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00,
    0x20, 0x3F, 0x20, 0x00, 0x00, 0x00,  // Y
    0x10, 0x08, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x20, 0x38,
    0x26, 0x21, 0x20, 0x20, 0x18, 0x00,  // Z
    0x00, 0x00, 0x00, 0xFE, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x00,  // [
    0x00, 0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x40, 0x40, 0x7F, 0x00, 0x00, 0x00,  // ]
    0x00, 0x00, 0x04, 0x02, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ^
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80,  // _
    0x00, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // `
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x19,
    0x24, 0x22, 0x22, 0x22, 0x3F, 0x20,  // a
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x3F,
    0x11, 0x20, 0x20, 0x11, 0x0E, 0x00,  // b
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x0E,
    0x11, 0x20, 0x20, 0x20, 0x11, 0x00,  // c
    0x00, 0x00, 0x00, 0x80, 0x80, 0x88, 0xF8, 0x00, 0x00, 0x0E,
    0x11, 0x20, 0x20, 0x10, 0x3F, 0x20,  // d
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F,
    0x22, 0x22, 0x22, 0x22, 0x13, 0x00,  // e
    0x00, 0x80, 0x80, 0xF0, 0x88, 0x88, 0x88, 0x18, 0x00, 0x20,
    0x20, 0x3F, 0x20, 0x20, 0x00, 0x00,  // f
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x6B,
    0x94, 0x94, 0x94, 0x93, 0x60, 0x00,  // g
    0x08, 0xF8, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F,
    0x21, 0x00, 0x00, 0x20, 0x3F, 0x20,  // h
    0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
    0x20, 0x3F, 0x20, 0x20, 0x00, 0x00,  // i
    0x00, 0x00, 0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0xC0,
    0x80, 0x80, 0x80, 0x7F, 0x00, 0x00,  // j
    0x08, 0xF8, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F,
    0x24, 0x02, 0x2D, 0x30, 0x20, 0x00,  // k
    0x00, 0x08, 0x08, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
    0x20, 0x3F, 0x20, 0x20, 0x00, 0x00,  // l
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F,
    0x20, 0x00, 0x3F, 0x20, 0x00, 0x3F,  // m
    0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F,
    0x21, 0x00, 0x00, 0x20, 0x3F, 0x20,  // n
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F,
    0x20, 0x20, 0x20, 0x20, 0x1F, 0x00,  // o
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0xFF,
    0xA1, 0x20, 0x20, 0x11, 0x0E, 0x00,  // p
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x0E,
    0x11, 0x20, 0x20, 0xA0, 0xFF, 0x80,  // q
    0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x20,
    0x3F, 0x21, 0x20, 0x00, 0x01, 0x00,  // r
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x33,
    0x24, 0x24, 0x24, 0x24, 0x19, 0x00,  // s
    0x00, 0x80, 0x80, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1F, 0x20, 0x20, 0x00, 0x00,  // t
    0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x1F,
    0x20, 0x20, 0x20, 0x10, 0x3F, 0x20,  // u
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x01,
    0x0E, 0x30, 0x08, 0x06, 0x01, 0x00,  // v
    0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x80, 0x0F, 0x30,
    0x0C, 0x03, 0x0C, 0x30, 0x0F, 0x00,  // w
    0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20,
    0x31, 0x2E, 0x0E, 0x31, 0x20, 0x00,  // x
    0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x81,
    0x8E, 0x70, 0x18, 0x06, 0x01, 0x00,  // y
    0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x21,
    0x30, 0x2C, 0x22, 0x21, 0x30, 0x00,  // z
    0x00, 0x00, 0x00, 0x00, 0x80, 0x7C, 0x02, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3F, 0x40, 0x40,  // {
    0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,  // |
    0x00, 0x02, 0x02, 0x7C, 0x80, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x40, 0x3F, 0x00, 0x00, 0x00, 0x00,  // }
    0x00, 0x06, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ~
};
