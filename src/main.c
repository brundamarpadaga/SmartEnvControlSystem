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

/* Include necessary headers */
#include "main.h"


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

    // OLED display struct
    static OLED_t oled;

    // update oled struct with I2C instance and semaphore
    // passed to task
    oled.iic = &IicInstance;
    oled.sem = xSemaphoreCreateBinary ( );
    if (oled.sem == NULL)
    {
        xil_printf ( "[ERROR] OLED semaphore creation failed\r\n" );
        return -1;  // Exit if semaphore creation fails
    }
    xSemaphoreGive ( oled.sem );
    oled_init(&oled);
    

    // Read BME280 calibration data for accurate measurements
    if ( bme_read_calibration_data ( &IicInstance, &calib_data ) != 0 )
    {
        xil_printf ( "[ERROR] BME280 calibration data read failed\r\n" );
        return -1;  // Exit if calibration data read fails
    }

    // Create Semaphore
    vSemaphoreCreateBinary ( binary_sem );
    vSemaphoreCreateBinary (bme280_sem);

    /* Create the queue */
    toPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint16_t));
    fromPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint32_t));

    /* Sanity check that the queue was created. */
    configASSERT (toPID);
    configASSERT (fromPID);

    /*Creat structure for bme280 data*/
    static sensor_Data sensorData;

    // Create task to parse user inputs
    xTaskCreate ( Parse_Input_Task, (const char*) "Parse_Input", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    // Create PID task for controlling light intensity
    xTaskCreate ( PID_Task, "PID", configMINIMAL_STACK_SIZE, &sensorData, 1, NULL );

    // Create task for displaying values to 7-segment display
    xTaskCreate ( Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    // Create BME280 task collects environment data (temp, pressure, humidity)
    xTaskCreate ( BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 2, &sensorData, 1, NULL );  // BME280 sensor task (larger stack)

    // Create task for displaying values to OLED display
    xTaskCreate ( OLED_Display_Task, "OLED", configMINIMAL_STACK_SIZE, &oled, 2, NULL );
    
    // Start the Scheduler
    xil_printf ( "Starting the scheduler\r\n" );

    vTaskStartScheduler ( );

    return -1;
}

static void prvSetupHardware ( void )
{
    uint32_t xStatus;

    const unsigned char ucSetToInput = 0xFFU;

    xil_printf ( "Initializing GPIO's\r\n" );

    /* Initialize the GPIO for the button inputs. */
    xStatus = XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );
    if ( xStatus == XST_SUCCESS )
    {
        /* Install the handler defined in this task for the button input.
        *NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
        must be used for this purpose. */
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr,
            NULL );

        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );

            /* Set switches and buttons to input. */
            XGpio_SetDataDirection ( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
            XGpio_SetDataDirection ( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

            /* Enable the button input interrupts in the interrupt controller.
            *NOTE* The vPortEnableInterrupt() API function must be used for this
            purpose. */

            vPortEnableInterrupt (
                XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR );

            /* Enable GPIO channel interrupts on button channel. Can moodify to
             * include switches */
            XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
        }
    }
    configASSERT ( ( xStatus == pdPASS ) );
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
    // Flag for context switch necessity
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
    
    // Signal input task that an interrupt occurred
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    
    // Clear GPIO interrupt
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
    
    // Yield to higher-priority task if needed
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
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

    uint8_t btns = 0x00;
    uint8_t sws = 0x00;
    uint16_t ValueToSend = 0x0000;

    while ( 1 )
        if ( xSemaphoreTake ( binary_sem, 500 ) )
        {
            ValueToSend &= 0x0000 ; // make sure old data is cleared
            btns = (NX4IO_getBtns() & 0x1F); // get btns and mask for c/u/d/l/r
            sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // get lower 8 switches
            ValueToSend |= ((btns << 8) | (sws)); // move btns to upper byte
            NX4IO_setLEDs(sws);
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );
        }
        else
        {
            // switch values checked even if buttons aren't pressed
            // and sends them to PID task if they change
            sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // get lower 8 switches
            if ((sws & ValueToSend) != sws)
            {
                ValueToSend &= 0x0000 ; // make sure old data is cleared
                ValueToSend |= (sws);
                NX4IO_setLEDs(sws);
                xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );
            }
            vTaskDelay ( 500 ); // delay to prevent task from running too fast and wasting resources
        }
}

/****************************************************************************
 * initialize the system
 *
 * This function is executed once at start-up and after resets.  It initializes
 * the peripherals and registers the interrupt handler(s)
 *****************************************************************************/
int do_init ( void )
{
    int status; // status from Xilinx Lib calls

    // initialize the Nexys4IO and Pmod544IO hardware and drivers
    // rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE
    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        return XST_FAILURE;
    }

    // Get AXI I2C device configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        return XST_FAILURE;
    }
    // Initialize the I2C driver
    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    // Enable the I2C Controller
    XIic_Start ( &IicInstance );

    // enable all rgb leds, these will be used for controlling PWM devices
    NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
    NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
    return XST_SUCCESS;
}

/**************************PID Task******************************************
*   Task Handles the Following:
*   Reads perameter message from MsgQ
*   Update new control/setpoint parameters
*   Get Current lux readig from TSL2561 sensor
*       done using the TSL2561 driver in implemented
*       in the C file of the same name
*   Execute PID algo function 
*   Drive PWM signal for LED, use RGB writ commands
*   write to display thread MsgQ to update
*   setpoint and current lux
*****************************************************************************/
void PID_Task (void* p)
{
    sensor_Data* sensorData = (sensor_Data*)p; // get the BME280 struct passed from main
    float luxOUT = 0;       // float percent value returned for LUX
    float tempOUT = 0;      // float percent value returned for temp
    float humOUT = 0;       // float percent value returned for hum
    uint16_t tsl2561 = 0;    // value returned from tsl2561 driver
    uint8_t envLight = 127; // 8-bit int value for controling "evironment lights"
    uint8_t envTemp = 127;  // 8-bit int value for controling "evironment temp"
    uint8_t envHum = 127;   // 8-bit int value for controling "evironment humidity"
    uint16_t btnSws;        // value recieved from the input task Q
    uint8_t btns = 0x10;    // btn values parsed from btnSws, start with center button "pressed"
    uint8_t sws;            // switch values parsed from btnSws
    TickType_t lastLuxTick = xTaskGetTickCount();// used for more accurate delta t values
    TickType_t lastBMETick = xTaskGetTickCount(); // used for sampling frequency of BME280
    static bool isInitialized = false;	// true if the init function has run at least once

    // create PID structs for temperature, humidity, and lux control
    static PID_t pidLux, pidTemp, pidHum;
    
    // initialize the pid struct if it hasn't been
    if(!isInitialized)
    {
        isInitialized = (pid_init(&pidLux) && pid_init(&pidTemp) && pid_init(&pidHum));
        if (!isInitialized)
        {
            xil_printf("PID struct initialization failed\r\n");
        }
    }

    // main task loop
    while(1)
    {
    	// recieve message from input task, 16-bit uint that contains switch and button values
    	    if (xQueueReceive (toPID, &btnSws, mainDONT_BLOCK) == pdPASS)
    	    {
    	    	// parse values recieved from input task
                if (btnSws & 0x1300) // center, left, or right button pressed
                {
                    btns = (btnSws & 0x1F00) >> 8;
                }
                else // get the new buttons presses and preserve last c/l/r press
                {
                    btns = ((btnSws & 0x1F00) >> 8) | 0x13; 
                }
                // get the new switch values
    	    	sws = (btnSws & 0x0FF);
    	    }
    	    else
    	    {
                // btn values should retain the last btnc/bntl/btnr value that was pressed
                // sws values should retain the last switch values
    	    	btns &= 0x13;
    	    	sws = sws;
    	    }

    	    // Get TSL2561 reading every 420ms and run PID algorithm for lux
            TickType_t currentLuxTick = xTaskGetTickCount();
            if (currentLuxTick - lastLuxTick >= 42)
            {
                float ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0); // visible and infrared
                float ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1 ); // just infrared
                tsl2561 = (uint16_t)ch0 - ((uint16_t)ch1 * 0.5); // calculate lux value
                
                // update delta_t for PID calculations
                pidLux.delta_t = ((currentLuxTick - lastLuxTick) * (1/100.0f));
                // update last tick time for use in next dt calculation
                lastLuxTick = currentLuxTick;

                // get correction percentage for lux
                luxOUT  = pid_funct(&pidLux, (int32_t)tsl2561);
            }	    

            // Get BME280 reading when semaphore is available and run PID algorithm for temp and humidity
            TickType_t currentBMETick = xTaskGetTickCount();
            if (xSemaphoreTake(bme280_sem, mainDONT_BLOCK)) // get BME280 data aprox every 1 second
            {
                // get time between samples for temp and humidty
                pidHum.delta_t = ((currentBMETick - lastBMETick) * (1/100.0f));
                pidTemp.delta_t = ((currentBMETick - lastBMETick) * (1/100.0f));

                // update last tick time for use in next dt calculation
                lastBMETick = currentBMETick;

                // get humidty and temp correction percentages using values stored in BME280 struct
                humOUT  = pid_funct(&pidHum, (int32_t)sensorData->humidity);
                tempOUT = pid_funct(&pidTemp, sensorData->temperature);

                xSemaphoreGive(bme280_sem); // release semaphore after reading
            }

            /*  Check which button was pressed and adjust PID values accordingly
    	    *   Center button pressed, increment humidity setpoint by 1
            *   Left button pressed, increment temperature setpoint by 1
            *   Right button pressed, increment lux setpoint by 1
            *   Diretion of increment dependent on up down buttons
            */
           switch (btns & 0x13)
           {
           case 0x10: // center button pressed, 7-seg display humidity
                displayHelper (&pidHum, btns, sensorData->humidity/1024, 1024);
            break;
            case 0x02: // left button pressed, 7-seg display temp
                displayHelper (&pidTemp, btns, sensorData->temperature/100, 100);
            break;
            case 0x01: // right button pressed, 7-seg display lux
                displayHelper (&pidLux, btns, tsl2561, 1);
            break;
            default:
            break;
           }

    	    /*PWM Write
    	    *   use the PID output to adjust the duty cycle and write it to the
            *       -heater
            *       -light
            *       -fan
    	    *   if the percentage returned is positive, the intensity needs to be increased
    	    *   because the sensor is reading a value lower than the setpoint
    	    *   if it's negative the intensity needs to be decreased becasue the
    	    *   sensor is reading a value higher than the setpoint
    	    *   using NX4IO_RGBLED_setDutyCycle command
    	    */

    	    // update pwm signals for humidty, light, and temp control
    	    envLight = correctedSignal(envLight, luxOUT);
            envHum   = correctedSignal(envHum, humOUT);
            envTemp  = correctedSignal(envTemp, tempOUT);


    	    /*Write to fan, status lights, enviro lights, and heater LED
            *   NX4IO_RGBLED_setDutyCycle(RGBx, red, green, blue) 
            *   RGB1_Red Status Red LED
            *   RGB1_Green Status Green LED
            *   RGB1_Blue Status Yellow LED
            *   RGB2_Red Duty for heater (Red LED)
            *   RGB2_Green Duty for fan
            *   RGB2_Blue Duty for enviro lights
            * 
            *   First checks if both humidty and temp are within 5% pf setpoint
            *   if fails checks if noth out
            *   if that fails checks if just humidty is out
            *   if that fails sets just temp as out
            * 
            *   updates status LEDs with max or min duty cycles based
            *   if signal is within setpoint or not
            */
            if (((humOUT >= (0.0-0.05)) && (humOUT < (0.0 + 0.05)))
                && ((tempOUT >= (0.0-0.05)) && (tempOUT < (0.0 + 0.05))))
            {
                NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, max_duty, min_duty); // All green
            }
            else if (!((humOUT >= (0.0-0.05)) && (humOUT < (0.0 + 0.05)))
            && !((tempOUT >= (0.0-0.05)) && (tempOUT < (0.0 + 0.05))))
            {
                NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, max_duty);
            }
            else
            {
                if(!((humOUT >= (0.0-0.05)) && (humOUT < (0.0 + 0.05)))
                && ((tempOUT >= (0.0-0.05)) && (tempOUT < (0.0 + 0.05))))
                {
                    NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, max_duty);
                }
                else
                {
                    NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, min_duty);
                }
            }
            
            // check switchs 0-2 and determin what value to set fan on 
            // fan will be driven by PID output or predefined percentages of 
            // of 25, 50, and 75
            switch (sws & 0x07)
            {
                case 0x01: 
                    NX4IO_RGBLED_setDutyCycle(RGB2, envTemp, (uint8_t)(0.25 * max_duty), envLight);
                break;
                case 0x02:
                    NX4IO_RGBLED_setDutyCycle(RGB2, envTemp, (uint8_t)(0.50 * max_duty), envLight);
                break;
                case 0x04:
                    NX4IO_RGBLED_setDutyCycle(RGB2, envTemp, (uint8_t)(0.75 * max_duty), envLight);
                break;
                default:
                    NX4IO_RGBLED_setDutyCycle(RGB2, envTemp, (envHum + envTemp)/2, envLight);
                break;

            }

    	    //vTaskDelay(42);
    }
}

/************************Display Task****************************************
*   Gets lux and setpoint values from Q and updates 7-seg display
*****************************************************************************/
void Display_Task (void* p)
{
    uint32_t recievedLux;
    uint16_t setpnt = 0x0000, luxVal;
    while (1)
    {
        // recieve new sensor lux reading and setpoint values
        xQueueReceive (fromPID, &recievedLux, portMAX_DELAY);

        // make sure old values are cleared
        setpnt &= 0x0000;
        luxVal &= 0x0000;

        // parse values
        luxVal |= (recievedLux & lux_mask);
        setpnt |= ((recievedLux >> 16) & lux_mask);

        // write values to 7-seg display
        NX410_SSEG_setAllDigits(SSEGHI, (uint8_t)(setpnt/100),
        (uint8_t)((setpnt%100)/10), (uint8_t)((setpnt%100)%10), 
        CC_BLANK, DP_NONE);
        NX410_SSEG_setAllDigits(SSEGLO, (uint8_t)(luxVal/100),
        (uint8_t)((luxVal%100)/10), (uint8_t)((luxVal%100)%10),
        CC_BLANK, DP_NONE);
    }
}

/*********************PID Initialization*************************************
*   Initializing PID structure for use in the PI Task
*****************************************************************************/
bool pid_init (PID_t *pid)
{
    pid -> Kp = 0.7;
    pid -> Ki = 0.2;
    pid -> Kd = 0.01;
    pid -> setpoint = 100;
    pid -> integral = 0;
    pid -> prev_error = 0;
    pid -> delta_t = 1; // set to the worst case sampling time but will dynamically update in use

    // returns true after initializing PID structure 
    return true;
}

/*********************PID Algo*************************************
*   Take PID structure and lux value from TSL2561
*   returns float percentage value, used for writing to LED
*   PID output = P + I + D
*   P = Kp(e(t)), where Kp is proportional gain
*   I = Ki(integral(e(t)dt)) where Ki is the integral gain
*   D = Kd((de(t))/dt) where Kd is the derivative gain
*   e(t) is the error and is equal to the setpoint - measured value
*   the integral can be though of as the accumulation of e(t)'s
*   the derivative can be though of as the (delta e(t))/(delta_t)
*   delta_t is the time between samples
*****************************************************************************/
float pid_funct (PID_t* pid, int32_t lux_value)
{
	// limits for integral range
	uint16_t max_int =  1024;
	float min_int = -1024;

    // e(t), error at time of sample
    float error = pid->setpoint - lux_value;

    // proportional
    float Pterm;
    // set to zero if switch[0] is O, affectively disabling proportional control
    Pterm = pid->Kp * error;
    
    // update integral, and get integral term
    // clamps to prevent Iterm from generating a value that would exceed 50% pwm range
    pid -> integral += (error * pid->delta_t);
    if(pid->integral >= max_int)
    {
    	pid->integral = max_int;
    }
    else if (pid->integral <= min_int)
    {
    	pid->integral = min_int;
    }

    float Iterm;
    // set to zero if switch[1] is 0
    Iterm = pid->Ki * pid->integral;   

    // get derivative term
    float Dterm;
    // set to zero if switch[2] is 0
    Dterm = pid->Kd * ((error - pid->prev_error) / pid->delta_t);

    // update previous error value
    pid->prev_error = error;

    // return a percentage value to be used for setting the intensity of PWM LED
    return (Pterm + Iterm + Dterm) / (float)pid->setpoint;
}

/*********************PID Helper*************************************
*   Take the current actuator driver and modify based on the correction
*   percentage given from PID algo function
*   returns uint8_t duty cycle after applying PID correction percentage
*   to current duty cycle
*
*   @enviro: duty cycle that is currently being applied to actuator
*   @pidOut: percentage to correct duty cycle by given from PID function 
*   
*****************************************************************************/
uint8_t correctedSignal (uint8_t enviro, float pidOut)
{
    if ((enviro + (pidOut * max_duty)) >= max_duty)
    {
        enviro = max_duty;
    }
    else if ((enviro + (pidOut * max_duty)) <= min_duty)
    {
        enviro = min_duty;
    }
    else
    {
        enviro = (uint8_t)(enviro + (pidOut * max_duty));
    }
    return enviro;
}

/***********************7-Seg Display Helper********************************
*   Create the message to send to the 7-Seg display
*   
*   returns uint8_t duty cycle after applying PID correction percentage
*   to current duty cycle
*
*   @pid: Pointer to PID struct for desired sensor
*   @btns: button value sent to PID task, used to determine if increment or decrement
*   @sensorVal: Reading from sensor
*   @incr: Increment amount, this should be 1 * (scaling for sensor)
*   lux sensor has scaling of 1
*   temp sensor has scaling of 100
*   humidty sensor has scaling of 1024*   
*****************************************************************************/
void displayHelper (PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr)
{
    uint32_t message = 0x00000000; // make sure message is cleared before each use

    if (btns & 0x08) // up button pressed
    {
        pid->setpoint += incr;
    }
    else if (btns & 0x04) // down button pressed
    {
        pid->setpoint -= incr;
    }

    message |= ((sensorVal << 0) | ((pid->setpoint/incr) << 16)); // build mesasge for 7-seg
    xQueueSend(fromPID, &message, mainDONT_BLOCK); // send message to 7-seg
}
