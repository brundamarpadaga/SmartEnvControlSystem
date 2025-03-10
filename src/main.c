/*
 * main.c - Entry point for TSL2561-based PID control system
 *
 * Purpose: This file initializes the FreeRTOS environment, sets up the TSL2561
 *          sensor driver, and creates tasks for reading lux values, running a
 *          PID controller to adjust LED brightness, and handling user input.
 *          It serves as the starting point for the ECE 544 Project #2 system.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 *
 */

/* Include necessary headers */
#include "main.h"

int main(void)
{
    // Announcement
    xil_printf("Hello from FreeRTOS LUX PID Controller\r\n");

    // Initialize the HW
    prvSetupHardware();
    do_init();
    tsl2561_init(&IicInstance);

    // Create Semaphore
    vSemaphoreCreateBinary(binary_sem);

    /* Create the queue */
    toPID   = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
    fromPID = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

    /* Sanity check that the queue was created. */
    configASSERT(toPID);
    configASSERT(fromPID);

    /*Creat pid structure for keeping track of PID elements*/
    static PID_t ledPID;

    // Create Task1
    xTaskCreate(
        Parse_Input_Task, (const char*) "Parse_Input", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Create Task2
    xTaskCreate(PID_Task, "PID", configMINIMAL_STACK_SIZE, &ledPID, 1, NULL);

    // Create Task3
    xTaskCreate(Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Start the Scheduler
    xil_printf("Starting the scheduler\r\n");
    // xil_printf ( "Push Button to change the LED pattern\r\n\r\n" );
    vTaskStartScheduler();

    return -1;
}

static void prvSetupHardware(void)
{
    uint32_t xStatus;

    const unsigned char ucSetToInput = 0xFFU;

    xil_printf("Initializing GPIO's\r\n");

    /* Initialize the GPIO for the button inputs. */

    xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID);

    if (xStatus == XST_SUCCESS)
    {
        /* Install the handler defined in this task for the button input.
        *NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
        must be used for this purpose. */
        xStatus = xPortInstallInterruptHandler(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL);

        if (xStatus == pdPASS)
        {
            xil_printf("Buttons interrupt handler installed\r\n");

            /* Set switches and buttons to input. */
            XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, ucSetToInput);
            XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, ucSetToInput);

            /* Enable the button input interrupts in the interrupt controller.
            *NOTE* The vPortEnableInterrupt() API function must be used for this
            purpose. */

            vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);

            /* Enable GPIO channel interrupts on button channel. Can moodify to
             * include switches */
            XGpio_InterruptEnable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
            XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
        }
    }

    configASSERT((xStatus == pdPASS));
}

// ISR, to handle interrupt of GPIO btns
// Give a Semaphore
static void gpio_intr(void* pvUnused)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);

    XGpio_InterruptClear(&xInputGPIOInstance, XGPIO_IR_MASK);

    /* Yield to higher-priority tasks if necessary */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// A task which takes the Interrupt Semaphore and sends the btn/sw states to PID
// Task
void Parse_Input_Task(void* p)
{
    uint8_t  btns        = 0x00;
    uint8_t  sws         = 0x00;
    uint16_t ValueToSend = 0x0000;

    while (1)
        if (xSemaphoreTake(binary_sem, 500))
        {
            btns = (NX4IO_getBtns() & 0x1E);                  // get btns and mask for u/d/c/l
            sws  = (uint8_t) (NX4IO_getSwitches() & 0x00FF);  // get lower 8 switches
            ValueToSend |= ((btns << 8) | (sws));             // move btnc to bit 10, btnu to bit
                                                              // 9, and bntd to bit 8
            NX4IO_setLEDs(sws);
            xQueueSend(toPID, &ValueToSend, mainDONT_BLOCK);
            ValueToSend &= 0x0000;  // clear btn/sw values for next time
        }
        else
            ;  // do nothing if semaphore isn't given
}

/****************************************************************************
 * initialize the system
 *
 * This function is executed once at start-up and after resets.  It initializes
 * the peripherals and registers the interrupt handler(s)
 *****************************************************************************/
int do_init(void)
{
    int status;  // status from Xilinx Lib calls

    // initialize the Nexys4IO and Pmod544IO hardware and drivers
    // rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE
    status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // Get AXI I2C device configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig(I2C_DEV_ID_ADDR);
    if (ConfigPtr == NULL)
    {
        return XST_FAILURE;
    }
    // Initialize the I2C driver
    status = XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
    if (status != XST_SUCCESS)
    {
        return status;
    }

    XIic_Start(&IicInstance);
    // Enable the I2C Controller

    NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
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
void PID_Task(void* p)
{
    PID_t*     pid     = (PID_t*) p;            // get the PID struct passed from main
    float      pidOUT  = 0;                     // float percent value returned from PID algo
    uint16_t   tsl2561 = 0;                     // float value returned from tsl2561 driver
    uint8_t    pwmLED  = 127;                   // 8-bit int value for writing to PWM LED
    uint16_t   btnSws;                          // value recieved from the input task Q
    uint32_t   setpntLux;                       // value to send to the display Task Q
    uint8_t    btns;                            // btn values parsed from btnSws
    uint8_t    sws;                             // switch values parsed from btnSws
    float      baseID = 0.01;                   // base increment for ID tuning set to 0.01
    float      baseP  = 0.1;                    // base increment for Kp to 0.1
    uint8_t    baseSP = 1;                      // base increment for setpoint 1
    uint8_t    incScaling;                      // scaling factor based on switch values
    uint8_t    btnPrint = 0x00;                 // control for toggeling value prints
    TickType_t lastTick = xTaskGetTickCount();  // used for more accurate delta t values

    static bool isInitialized = false;  // true if the init function has run at least once
    // initialize the pid struct if it hasn't been
    if (!isInitialized)
    {
        isInitialized = pid_init(pid);
    }

    // main task loop
    while (1)
    {
        // recieve message from input task, 16-bit uint that contains switch and
        // button values
        if (xQueueReceive(toPID, &btnSws, 42) == pdPASS)
        {
            // parse values recieved from input task
            btns = (btnSws & 0x1E00) >> 8;
            sws  = (btnSws & 0x0FF);
        }
        else
        {
            btns = 0x00;
            sws  = sws;
        }

        // check left button and toggle print if pressed
        if (btns & 0x02)
        {
            btnPrint = !btnPrint;
        }

        // scale base increments based on values of switches[5:4]
        if (!(sws & 0x30))  // switches [5:4] = [0:0]
        {
            incScaling = 1;
        }
        else if (sws & 0x20)  // switches [5:4] = [1:X]
        {
            incScaling = 10;
        }
        else if (sws & 0x10)  // switches [5:4] = [1:0]
        {
            incScaling = 5;
        }

        /* PID modifier
         *   if switch[3] only increment or decrement setpoint, based on button
         * press else check switches[7:6] Just switch [6] = 0x40, Kp should
         * changed on button press Just switch [7] = 0x80, Ki should be changed
         * on button press Both switches [7:6] = 0XC0, Kd should be changed on
         * button press for buttons, a value of 1 means up button was pressed a
         * value of 2 means down button was pressed After parsing, updates the
         * PID struct with the setpoint or PID gain values has controls to
         * prevent setpoint from being set higher or lower than the upper and
         * lower limits Also prevents Kp, Ki, and Kd gains from going negative
         * when decrementing uses macro(UPDATE_SATURATING) for saturation
         * checks, help with readability and code redundancy
         */
        if ((sws & 0x08))
        {
            if (btns & 0x08)
            {
                UPDATE_SATURATING(
                    pid->setpoint, (incScaling * baseSP), pid->min_lim, pid->max_lim, true);
            }
            else if (btns & 0x04)
            {
                UPDATE_SATURATING(
                    pid->setpoint, (incScaling * baseSP), pid->min_lim, pid->max_lim, false);
            }
            else
            {
                pid->setpoint = pid->setpoint;
            }
        }
        else
        {
            switch (sws & 0xC0)
            {
                case 0x40:
                    if ((btns & 0x08))
                    {
                        UPDATE_SATURATING(pid->Kp, (incScaling * baseP), 0, pid->max_lim, true);
                    }
                    else if ((btns & 0x04))
                    {
                        UPDATE_SATURATING(pid->Kp, (incScaling * baseP), 0, pid->max_lim, false);
                    }
                    else
                    {
                        pid->Kp = pid->Kp;
                    }
                    break;

                case 0x80:
                    if ((btns & 0x08))
                    {
                        UPDATE_SATURATING(pid->Ki, (incScaling * baseID), 0, pid->max_lim, true);
                    }
                    else if ((btns & 0x04))
                    {
                        UPDATE_SATURATING(pid->Ki, (incScaling * baseID), 0, pid->max_lim, false);
                    }
                    else
                    {
                        pid->Ki = pid->Ki;
                    }
                    break;

                case 0xC0:
                    if ((btns & 0x08))
                    {
                        UPDATE_SATURATING(pid->Kd, (incScaling * baseID), 0, pid->max_lim, true);
                    }
                    else if ((btns & 0x04))
                    {
                        UPDATE_SATURATING(pid->Kd, (incScaling * baseID), 0, pid->max_lim, false);
                    }
                    else
                    {
                        pid->Kd = pid->Kd;
                    }
                    break;

                default:
                    break;
            }
        }

        // Get TSL2561 reading
        float ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);  // visible and infrared
        float ch1 = tsl2561_readChannel(&IicInstance,
                                        TSL2561_CHANNEL_1);   // just infrared
        tsl2561   = (uint16_t) ch0 - ((uint16_t) ch1 * 0.5);  // basic lux approx.

        // this may need to be moved into the TSL2561 module to result in a more
        // accurate reading get tick when sample is collected and use to
        // determin dt
        TickType_t currentTick = xTaskGetTickCount();
        pid->delta_t           = ((currentTick - lastTick) * (1 / 100.0f));
        // update last tick time for use in next dt calculation
        lastTick = currentTick;

        // running PID algorithm, uses float from TSL2561 driver
        // returns the percentage to increase/decrease the pwmLED by
        pidOUT = pid_funct(pid, tsl2561, sws);

        /*PWM LED Write
         *   use the PID output to adjust the duty cycle and write it to the PWM
         * LED if the percentage returned is positive, the intensity needs to be
         * increased because the sensor is reading a value lower than the
         * setpoint if it's negative the intensity needs to be decreased becasue
         * the sensor is reading a value hgiher than the setpoint End by writing
         * duty cycle to RGB1 blue, drives pwmLED using
         * NX4IO_RGBLED_setDutyCycle command
         */

        // clamp the output to the max value if pidOUT >= 1
        // clamp the output to the min value if the correction pulls output <=
        // min value otherwise adjusts pwmLED by the required percentage
        if ((pwmLED + (pidOUT * max_duty)) >= max_duty)
        {
            pwmLED = max_duty;
        }
        else if ((pwmLED + (pidOUT * max_duty)) <= min_duty)
        {
            pwmLED = min_duty;
        }
        else
        {
            pwmLED = (uint8_t) (pwmLED + (pidOUT * max_duty));
        }
        // send PWM value to LED
        NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED);

        // Print setpoint, lux value, and duty cycle over serial port. Used for
        // plotting
        if (btnPrint)
        {
            xil_printf("Setpoint Value: %d\r\n", pid->setpoint);
            xil_printf("Lux Value: %d\r\n", tsl2561);
            xil_printf("PWM LED Duty Cycle: %d\r\n", pwmLED);
        }

        // update setpntLux with value to be displayed by display task
        setpntLux = 0x00000000;  // make sure old data is cleared
        setpntLux |= (tsl2561 << 0) |
                     (pid->setpoint << 16);  //(uint32_t)((((uint32_t)(tsl2561) & lux_mask) << 0) |
                                             //// sensor measured value in lower 32 bits
                                             //(((uint32_t)(pid->setpoint) & lux_mask) << 16)); //
                                             // setpoint in upper 32 bits

        // send message to display thread MsgQ to update setpoint and current
        // lux
        xQueueSend(fromPID, &setpntLux, mainDONT_BLOCK);

        // if center button is pressed print out the current PID configuration
        if ((btns & 0x10))
        {
            print_pid(pid);
        }

        // vTaskDelay(42);
    }
}

/************************Display Task****************************************
 *   Gets lux and setpoint values from Q and updates 7-seg display
 *****************************************************************************/
void Display_Task(void* p)
{
    uint32_t recievedLux;
    uint16_t setpnt = 0x0000, luxVal;
    while (1)
    {
        // recieve new sensor lux reading and setpoint values
        xQueueReceive(fromPID, &recievedLux, portMAX_DELAY);

        // make sure old values are cleared
        setpnt &= 0x0000;
        luxVal &= 0x0000;

        // parse values
        luxVal |= (recievedLux & lux_mask);
        setpnt |= ((recievedLux >> 16) & lux_mask);

        // write values to 7-seg display
        NX410_SSEG_setAllDigits(SSEGHI,
                                (uint8_t) (setpnt / 100),
                                (uint8_t) ((setpnt % 100) / 10),
                                (uint8_t) ((setpnt % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
        NX410_SSEG_setAllDigits(SSEGLO,
                                (uint8_t) (luxVal / 100),
                                (uint8_t) ((luxVal % 100) / 10),
                                (uint8_t) ((luxVal % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
    }
}

/*********************PID Initialization*************************************
 *   Initializing PID structure for use in the PI Task
 *****************************************************************************/
bool pid_init(PID_t* pid)
{
    pid->Kp         = 0.7;
    pid->Ki         = 0.2;
    pid->Kd         = 0;
    pid->setpoint   = 100;
    pid->integral   = 0;
    pid->prev_error = 0;
    pid->delta_t    = 0.437;  // set to the worst case sampling time but will
                              // dynamically update in use
    // assumes that the float value returned is a 16 bit value, would need be
    // adjusted
    pid->max_lim = 1000;  // set max value to 99%, should result in setting LED PWM to 99%
    pid->min_lim = 0;     // set min value to 1%, should result in setting LED PWM to 1%

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
float pid_funct(PID_t* pid, uint16_t lux_value, uint8_t switches)
{
    // limits for integral range
    uint16_t max_int = 2048;
    float    min_int = -2048;

    // e(t), error at time of sample
    float error = pid->setpoint - lux_value;

    // proportional
    float Pterm;
    // set to zero if switch[0] is O, affectively disabling proportional control
    if (switches & 0x01)
    {
        Pterm = pid->Kp * error;
    }
    else
    {
        Pterm = 0;
    }

    // update integral, and get integral term
    // clamps to prevent Iterm from generating a value that would exceed 50% pwm
    // range
    pid->integral += (error * pid->delta_t);
    if (pid->integral >= max_int)
    {
        pid->integral = max_int;
    }
    else if (pid->integral <= min_int)
    {
        pid->integral = min_int;
    }

    float Iterm;
    // set to zero if switch[1] is 0
    if (switches & 0x02)
    {
        Iterm = pid->Ki * pid->integral;
    }
    else
    {
        Iterm = 0;
    }

    // get derivative term
    float Dterm;
    // set to zero if switch[2] is 0
    if (switches & 0x04)
    {
        Dterm = pid->Kd * ((error - pid->prev_error) / pid->delta_t);
    }
    else
    {
        Dterm = 0;
    }

    // update previous error value
    pid->prev_error = error;

    // return a percentage value to be used for setting the intensity of PWM LED
    return (Pterm + Iterm + Dterm) / (float) pid->setpoint;
}

// Prints out the PID gains when the center is pressed
void print_pid(PID_t* pid)
{
    xil_printf("PID gains:\r\n");
    xil_printf("Kp       = %u.%02u\r\n",
               (uint16_t) pid->Kp,
               (uint16_t) ((pid->Kp - (uint16_t) pid->Kp) * 100));
    xil_printf("Ki       = %u.%02u\r\n",
               (uint16_t) pid->Ki,
               (uint16_t) ((pid->Ki - (uint16_t) pid->Ki) *
                           100));  // float, manipulated because xil_printf
                                   // doesn't handle floating point
    xil_printf("Kd       = %u.%02u\r\n",
               (uint16_t) pid->Kd,
               (uint16_t) ((pid->Kd - (uint16_t) pid->Kd) *
                           100));  // float, manipulated because xil_printf
                                   // doesn't handle floating point
}
