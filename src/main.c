/*
 * main.c - TSL2561 and BME280-based PID control system with OLED
 *
 * Purpose: Multi-task system with semaphores for GPIO and BME280, optimized for 128KB BRAM.
 *
 * Course: ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "main.h"
#include <stdio.h>

/* Global variables defined here (declared extern in main.h) */
XGpio             xInputGPIOInstance;
XIic              IicInstance;
SemaphoreHandle_t binary_sem;
SemaphoreHandle_t bme280_sem;
xQueueHandle      toPID;   /* Not used but kept for main.h */
xQueueHandle      fromPID; /* Not used but kept for main.h */

/* Additional globals (minimized) */
static OLED_t            oled;
struct bme280_calib_data calib_data;
static volatile uint16_t btnSws;
static volatile uint16_t lux;
static volatile struct
{
    float    temp_f;
    uint32_t hum;
    uint32_t press;
} sensor_data;

#define STACK_SIZE 48 /* 192 bytes per task, reduced from 64 */

void prvSetupHardware(void)
{
    uint32_t xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (xStatus == XST_SUCCESS)
    {
        xStatus = xPortInstallInterruptHandler(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL);
        if (xStatus == pdPASS)
        {
            XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, 0xFF);
            XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, 0xFF);
            vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);
            XGpio_InterruptEnable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
            XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
        }
    }
}

static void gpio_intr(void* pvUnused)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);
    XGpio_InterruptClear(&xInputGPIOInstance, XGPIO_IR_MASK);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int do_init(void)
{
    if (NX4IO_initialize(N4IO_BASEADDR) != XST_SUCCESS)
        return XST_FAILURE;
    XIic_Config* ConfigPtr = XIic_LookupConfig(I2C_DEV_ID_ADDR);
    if (ConfigPtr == NULL)
        return XST_FAILURE;
    if (XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;
    XIic_Start(&IicInstance);
    NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
    return XST_SUCCESS;
}

bool pid_init(PID_t* pid)
{
    pid->Kp         = 0.7;
    pid->Ki         = 0.2;
    pid->Kd         = 0;
    pid->setpoint   = 100;
    pid->integral   = 0;
    pid->prev_error = 0;
    pid->max_lim    = 1000;
    pid->min_lim    = 0;
    return true;
}

float pid_funct(PID_t* pid, uint16_t lux_value, uint8_t switches)
{
    float error = pid->setpoint - lux_value;
    float Pterm = (switches & 0x01) ? pid->Kp * error : 0;
    pid->integral += error;
    if (pid->integral > 2048)
        pid->integral = 2048;
    else if (pid->integral < -2048)
        pid->integral = -2048;
    float Iterm     = (switches & 0x02) ? pid->Ki * pid->integral : 0;
    float Dterm     = (switches & 0x04) ? pid->Kd * (error - pid->prev_error) : 0;
    pid->prev_error = error;
    return (Pterm + Iterm + Dterm) / pid->setpoint;
}

void print_pid(PID_t* pid)
{
    /* Empty to minimize size */
}

void Parse_Input_Task(void* p)
{
    while (1)
    {
        if (xSemaphoreTake(binary_sem, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            uint8_t btns = NX4IO_getBtns() & 0x1E;
            uint8_t sws  = NX4IO_getSwitches() & 0xFF;
            btnSws       = (btns << 8) | sws;
            NX4IO_setLEDs(sws);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void PID_Task(void* p)
{
    PID_t*  pid    = (PID_t*) p;
    uint8_t pwmLED = 127;
    pid_init(pid);

    while (1)
    {
        uint8_t sws = btnSws & 0xFF;
        float   ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
        float   ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
        lux         = (uint16_t) ch0 - ((uint16_t) ch1 * 0.5);

        float pidOUT = pid_funct(pid, lux, sws);
        if (pwmLED + (pidOUT * max_duty) > max_duty)
            pwmLED = max_duty;
        else if (pwmLED + (pidOUT * max_duty) < min_duty)
            pwmLED = min_duty;
        else
            pwmLED += (uint8_t) (pidOUT * max_duty);
        NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void OLED_Display_Task(void* p)
{
    while (1)
    {
        oled_write_string(&oled, "T: -- F", 0);
        oled_write_string(&oled, "H: -- %", 2);
        oled_write_string(&oled, "P: -- Pa", 4);
        oled_write_string(&oled, "L: -- lux", 6);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void BME280_Task(void* pvParameters)
{
    uint8_t buffer[8];
    buffer[0] = REG_CTRL_HUM;
    buffer[1] = 0x01;
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);
    buffer[0] = REG_CTRL_MEAS;
    buffer[1] = (0x5 << 5) | (0x1 << 2) | 0x3;
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);
    buffer[0] = REG_CONFIG;
    buffer[1] = (0x4 << 2) | (0x5 << 5);
    XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP);
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        if (xSemaphoreTake(bme280_sem, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            buffer[0] = REG_DATA;
            XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
            XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP);

            struct bme280_uncomp_data uncomp_data;
            uncomp_data.pressure =
                ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
            uncomp_data.temperature =
                ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
            uncomp_data.humidity = ((uint32_t) buffer[6] << 8) | buffer[7];

            int32_t temp_c     = compensate_temperature(&uncomp_data, &calib_data);
            sensor_data.temp_f = (temp_c / 100.0f) * 9 / 5 + 32;
            sensor_data.press  = compensate_pressure(&uncomp_data, &calib_data);
            sensor_data.hum    = compensate_humidity(&uncomp_data, &calib_data);

            xSemaphoreGive(bme280_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    prvSetupHardware();
    if (do_init() != XST_SUCCESS)
        while (1)
            ;
    if (tsl2561_init(&IicInstance) != XST_SUCCESS)
        while (1)
            ;
    if (bme_init(&IicInstance) != 0)
        while (1)
            ;
    if (bme_read_calibration_data(&IicInstance, &calib_data) != 0)
        while (1)
            ;

    oled.iic = &IicInstance;
    oled.sem = xSemaphoreCreateBinary();
    if (oled.sem == NULL)
        while (1)
            ;
    xSemaphoreGive(oled.sem);
    oled_init(&oled);

    vSemaphoreCreateBinary(binary_sem);
    if (binary_sem == NULL)
        while (1)
            ;

    bme280_sem = xSemaphoreCreateBinary();
    if (bme280_sem == NULL)
        while (1)
            ;
    xSemaphoreGive(bme280_sem);

    toPID   = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
    fromPID = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    if (!toPID || !fromPID)
        while (1)
            ;

    static PID_t ledPID;
    xTaskCreate(Parse_Input_Task, "Input", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(PID_Task, "PID", STACK_SIZE, &ledPID, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(OLED_Display_Task, "OLED", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(BME280_Task, "BME280", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    while (1)
        ;
    return -1;
}
