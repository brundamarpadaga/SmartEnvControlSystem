/*
 * main.c - Entry point for TSL2561 and BME280-based PID control system
 *
 * Purpose: This file initializes the FreeRTOS environment, sets up the TSL2561
 *          and BME280 sensors, and creates tasks for reading lux, temperature,
 *          pressure, and humidity values, running a PID controller to adjust
 *          LED brightness, and handling user input and display updates.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "main.h"

#include <stdlib.h>

int main ( void )
{
    xil_printf ( "Hello from FreeRTOS LUX and BME280 PID Controller\r\n" );

    prvSetupHardware ( );
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Hardware initialization failed\r\n" );
        return -1;
    }

    tsl2561_init ( &IicInstance );
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

    vSemaphoreCreateBinary ( binary_sem );
    bme280_sem = xSemaphoreCreateBinary ( );
    if ( bme280_sem == NULL )
    {
        xil_printf ( "[ERROR] BME280 semaphore creation failed\r\n" );
        return -1;
    }
    xSemaphoreGive ( bme280_sem );

    toPID   = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );
    fromPID = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint32_t ) );
    configASSERT ( toPID );
    configASSERT ( fromPID );

    static PID_t ledPID;

    xTaskCreate ( Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate ( PID_Task, "PID", configMINIMAL_STACK_SIZE, &ledPID, 1, NULL );
    xTaskCreate ( Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate ( BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL );

    xil_printf ( "Starting the scheduler\r\n" );
    vTaskStartScheduler ( );

    return -1;
}

static void prvSetupHardware ( void )
{
    uint32_t            xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xil_printf ( "Initializing GPIO's\r\n" );

    xStatus = XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );
    if ( xStatus == XST_SUCCESS )
    {
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL );
        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );
            XGpio_SetDataDirection ( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
            XGpio_SetDataDirection ( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );
            vPortEnableInterrupt ( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR );
            XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
        }
    }
    configASSERT ( ( xStatus == pdPASS ) );
}

static void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
}

int do_init ( void )
{
    int status;

    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        return XST_FAILURE;
    }

    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        return XST_FAILURE;
    }

    status = XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    XIic_Start ( &IicInstance );
    NX4IO_RGBLED_setChnlEn ( RGB1, false, false, true );
    return XST_SUCCESS;
}

void Parse_Input_Task ( void* p )
{
    uint8_t  btns        = 0x00;
    uint8_t  sws         = 0x00;
    uint16_t ValueToSend = 0x0000;

    while ( 1 )
    {
        if ( xSemaphoreTake ( binary_sem, 500 ) )
        {
            btns = ( NX4IO_getBtns ( ) & 0x1E );
            sws  = (uint8_t) ( NX4IO_getSwitches ( ) & 0x00FF );
            ValueToSend |= ( ( btns << 8 ) | sws );
            NX4IO_setLEDs ( sws );
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );
            ValueToSend &= 0x0000;
        }
    }
}

void PID_Task ( void* p )
{
    PID_t*      pid     = (PID_t*) p;
    float       pidOUT  = 0;
    uint16_t    tsl2561 = 0;
    uint8_t     pwmLED  = 127;
    uint16_t    btnSws;
    uint32_t    setpntLux;
    uint8_t     btns, sws;
    float       baseID = 0.01, baseP = 0.1;
    uint8_t     baseSP        = 1, incScaling;
    uint8_t     btnPrint      = 0x00;
    TickType_t  lastTick      = xTaskGetTickCount ( );
    static bool isInitialized = false;

    if ( !isInitialized )
    {
        isInitialized = pid_init ( pid );
    }

    while ( 1 )
    {
        if ( xQueueReceive ( toPID, &btnSws, 42 ) == pdPASS )
        {
            btns = ( btnSws & 0x1E00 ) >> 8;
            sws  = ( btnSws & 0x0FF );
        }
        else
        {
            btns = 0x00;
        }

        if ( btns & 0x02 ) btnPrint = !btnPrint;

        if ( !( sws & 0x30 ) )
            incScaling = 1;
        else if ( sws & 0x20 )
            incScaling = 10;
        else if ( sws & 0x10 )
            incScaling = 5;

        if ( sws & 0x08 )
        {
            if ( btns & 0x08 )
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, true );
            else if ( btns & 0x04 )
                UPDATE_SATURATING (
                    pid->setpoint, ( incScaling * baseSP ), pid->min_lim, pid->max_lim, false );
        }
        else
        {
            switch ( sws & 0xC0 )
            {
                case 0x40:
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Kp, ( incScaling * baseP ), 0, pid->max_lim, false );
                    break;
                case 0x80:
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Ki, ( incScaling * baseID ), 0, pid->max_lim, false );
                    break;
                case 0xC0:
                    if ( btns & 0x08 )
                        UPDATE_SATURATING (
                            pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, true );
                    else if ( btns & 0x04 )
                        UPDATE_SATURATING (
                            pid->Kd, ( incScaling * baseID ), 0, pid->max_lim, false );
                    break;
            }
        }

        float ch0 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_0 );
        float ch1 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_1 );
        tsl2561   = (uint16_t) ch0 - ( (uint16_t) ch1 * 0.5 );

        TickType_t currentTick = xTaskGetTickCount ( );
        pid->delta_t           = ( ( currentTick - lastTick ) * ( 1 / 100.0f ) );
        lastTick               = currentTick;

        pidOUT = pid_funct ( pid, tsl2561, sws );

        if ( ( pwmLED + ( pidOUT * max_duty ) ) >= max_duty )
            pwmLED = max_duty;
        else if ( ( pwmLED + ( pidOUT * max_duty ) ) <= min_duty )
            pwmLED = min_duty;
        else
            pwmLED = (uint8_t) ( pwmLED + ( pidOUT * max_duty ) );

        NX4IO_RGBLED_setDutyCycle ( RGB1, 0, 0, pwmLED );

        if ( btnPrint )
        {
            xil_printf ( "Setpoint Value: %d\r\n", pid->setpoint );
            xil_printf ( "Lux Value: %d\r\n", tsl2561 );
            xil_printf ( "PWM LED Duty Cycle: %d\r\n", pwmLED );
        }

        setpntLux = ( tsl2561 << 0 ) | ( pid->setpoint << 16 );
        xQueueSend ( fromPID, &setpntLux, mainDONT_BLOCK );

        if ( btns & 0x10 ) print_pid ( pid );
    }
}

void Display_Task ( void* p )
{
    uint32_t recievedLux;
    uint16_t setpnt = 0x0000, luxVal;
    while ( 1 )
    {
        xQueueReceive ( fromPID, &recievedLux, portMAX_DELAY );
        luxVal = ( recievedLux & lux_mask );
        setpnt = ( ( recievedLux >> 16 ) & lux_mask );
        NX410_SSEG_setAllDigits ( SSEGHI,
                                  ( setpnt / 100 ),
                                  ( ( setpnt % 100 ) / 10 ),
                                  ( ( setpnt % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
        NX410_SSEG_setAllDigits ( SSEGLO,
                                  ( luxVal / 100 ),
                                  ( ( luxVal % 100 ) / 10 ),
                                  ( ( luxVal % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
    }
}

bool pid_init ( PID_t* pid )
{
    pid->Kp         = 0.7;
    pid->Ki         = 0.2;
    pid->Kd         = 0;
    pid->setpoint   = 100;
    pid->integral   = 0;
    pid->prev_error = 0;
    pid->delta_t    = 0.437;
    pid->max_lim    = 1000;
    pid->min_lim    = 0;
    return true;
}

float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches )
{
    uint16_t max_int = 2048;
    float    min_int = -2048;
    float    error   = pid->setpoint - lux_value;
    float    Pterm   = ( switches & 0x01 ) ? pid->Kp * error : 0;
    pid->integral += ( error * pid->delta_t );
    if ( pid->integral >= max_int )
        pid->integral = max_int;
    else if ( pid->integral <= min_int )
        pid->integral = min_int;
    float Iterm = ( switches & 0x02 ) ? pid->Ki * pid->integral : 0;
    float Dterm =
        ( switches & 0x04 ) ? pid->Kd * ( ( error - pid->prev_error ) / pid->delta_t ) : 0;
    pid->prev_error = error;
    return ( Pterm + Iterm + Dterm ) / (float) pid->setpoint;
}

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
