
/*
 * ===========================================================================
 * main.c - Entry point for TSL2561 and BME280-based PID control system
 * ===========================================================================
 * Purpose: Initializes the FreeRTOS environment, configures TSL2561 and BME280
 *          sensors, and creates tasks for environmental monitoring and control.
 *          Handles lux, temperature, pressure, and humidity readings, runs a
 *          PID controller to adjust actuators, and manages user input and
 * displays.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Final Project - Environmental Control System
 */

/* Include necessary headers */
#include "main.h"

/* ===========================================================================
 * Global Variable Definitions
 * ===========================================================================
 */
XIic IicInstance;                       // I2C instance for sensor communication
struct bme280_calib_data calib_data;    // BME280 sensor calibration data
SemaphoreHandle_t binary_sem = NULL;    // Semaphore for GPIO interrupt sync
SemaphoreHandle_t bme280_sem = NULL;    // Semaphore for BME280 access
SemaphoreHandle_t oled_sem = NULL;      // Semaphore for OLED access
SemaphoreHandle_t i2c_sem = NULL;       // Semaphore for I2C bus access
XGpio xInputGPIOInstance;               // GPIO instance for buttons/switches
QueueHandle_t toPID = NULL;             // Queue for input data to PID task
QueueHandle_t fromPID = NULL;           // Queue for display data from PID task
QueueHandle_t i2c_request_queue = NULL; // Queue for I2C requests

/* ===========================================================================
 * Macro Definitions
 * ===========================================================================
 */

/**
 * @brief Logging macro for conditional debug output.
 * @details Prints messages via xil_printf if level <= DEBUG_LEVEL.
 */
#define LOG(level, fmt, ...)                                                   \
  if (level <= DEBUG_LEVEL)                                                    \
  xil_printf(fmt, ##__VA_ARGS__)

/* ===========================================================================
 * Function Definitions
 * ===========================================================================
 */

/**
 * @brief Program entry point.
 * @details Initializes hardware, sensors, FreeRTOS primitives, and tasks.
 * @return int Never returns in FreeRTOS; -1 on initialization failure.
 */
int main(void) {
  xil_printf("Hello from FreeRTOS LUX and BME280 PID Controller\r\n");

  prvSetupHardware();

  if (do_init() != XST_SUCCESS) {
    xil_printf("[ERROR] Hardware initialization failed\r\n");
    cleanup_system();
    return -1;
  }

  i2c_sem = xSemaphoreCreateBinary();
  if (i2c_sem == NULL) {
    xil_printf("[ERROR] I2C semaphore creation failed\r\n");
    cleanup_system();
    return -1;
  }
  xSemaphoreGive(i2c_sem);

  if (tsl2561_init(&IicInstance) != XST_SUCCESS) {
    xil_printf("[ERROR] TSL2561 initialization failed\r\n");
    cleanup_system();
    return -1;
  }

  if (bme_init(&IicInstance) != 0 ||
      bme_read_calibration_data(&IicInstance, &calib_data) != 0) {
    xil_printf("[ERROR] BME280 initialization or calibration failed\r\n");
    cleanup_system();
    return -1;
  }

  if (lcd_init(&IicInstance) != XST_SUCCESS) {
    xil_printf("[ERROR] OLED initialization failed\r\n");
    cleanup_system();
    return -1;
  }

  binary_sem = xSemaphoreCreateBinary();
  if (binary_sem == NULL) {
    xil_printf("[ERROR] GPIO semaphore creation failed\r\n");
    cleanup_system();
    return -1;
  }
  xSemaphoreGive(binary_sem);

  bme280_sem = xSemaphoreCreateBinary();
  if (bme280_sem == NULL) {
    xil_printf("[ERROR] BME280 semaphore creation failed\r\n");
    cleanup_system();
    return -1;
  }
  xSemaphoreGive(bme280_sem);

  oled_sem = xSemaphoreCreateBinary();
  if (oled_sem == NULL) {
    xil_printf("[ERROR] OLED semaphore creation failed\r\n");
    cleanup_system();
    return -1;
  }
  xSemaphoreGive(oled_sem);

  i2c_request_queue = xQueueCreate(10, sizeof(i2c_request_t));
  toPID = xQueueCreate(mainQUEUE_LENGTH + 1, sizeof(uint16_t));
  fromPID = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
  if (!i2c_request_queue || !toPID || !fromPID) {
    xil_printf("[ERROR] Queue creation failed\r\n");
    cleanup_system();
    return -1;
  }

  static sensor_Data sensorData; // Assuming sensor_Data is defined in pidtask.h

  xTaskCreate(Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE, NULL,
              3, NULL);
  xTaskCreate(PID_Task, "PID", configMINIMAL_STACK_SIZE, &sensorData, 1, NULL);
  xTaskCreate(Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 2, &sensorData,
              1, NULL);
  xTaskCreate(LCD_Task, "OLED", configMINIMAL_STACK_SIZE * 4, &sensorData, 2,
              NULL);
  xTaskCreate(I2C_Task, "I2C", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  xil_printf("Starting the scheduler\r\n");
  vTaskStartScheduler();

  cleanup_system();
  return -1;
}

/**
 * @brief Configures GPIO and interrupt system.
 * @details Enables interrupts for both buttons and switches.
 */
void prvSetupHardware(void) {
  uint32_t xStatus;
  const unsigned char ucSetToInput = 0xFFU;

  xil_printf("Initializing GPIO's\r\n");

  xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID);
  if (xStatus == XST_SUCCESS) {
    xStatus = xPortInstallInterruptHandler(
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr,
        NULL);
    if (xStatus == pdPASS) {
      xil_printf("Buttons and switches interrupt handler installed\r\n");
      XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, ucSetToInput);
      XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, ucSetToInput);
      vPortEnableInterrupt(
          XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);
      XGpio_InterruptEnable(&xInputGPIOInstance,
                            XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
      XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
    }
  }
  configASSERT(xStatus == pdPASS);
}

/**
 * @brief GPIO interrupt service routine.
 * @param pvUnused Unused parameter.
 */
void gpio_intr(void *pvUnused __attribute__((unused))) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (binary_sem != NULL) {
    xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);
  }

  XGpio_InterruptClear(&xInputGPIOInstance,
                       XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Task to process user inputs.
 * @details Simplified to match the old working version with minimal debouncing.
 * @param p Unused parameter.
 */
void Parse_Input_Task(void *p __attribute__((unused))) {
  uint8_t btns = 0x00, last_btns = 0x00;
  uint8_t sws = 0x00, last_sws = 0x00;
  uint16_t ValueToSend = 0x0000;
  TickType_t last_debounce = xTaskGetTickCount();
  const TickType_t debounce_delay = pdMS_TO_TICKS(20); // 20ms debounce period

  while (1) {
    if (xSemaphoreTake(binary_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
      TickType_t now = xTaskGetTickCount();
      if (now - last_debounce >= debounce_delay) {
        btns = NX4IO_getBtns() & BUTTON_MASK;
        sws = (uint8_t)(NX4IO_getSwitches() & SWITCH_MASK);
        if (btns != last_btns || sws != last_sws) {
          ValueToSend = (btns << 8) | sws;
          NX4IO_setLEDs(sws);
          if (xQueueSend(toPID, &ValueToSend, QUEUE_TIMEOUT_TICKS) != pdPASS) {
            LOG(LOG_LEVEL_WARNING, "[WARNING] Queue to PID full\r\n");
          }
          LOG(LOG_LEVEL_INFO,
              "[INFO] Input updated: btns=0x%02x, sws=0x%02x\r\n", btns, sws);
          last_btns = btns;
          last_sws = sws;
          last_debounce = now;
        }
      }
    }
  }
}

/**
 * @brief Initializes peripherals (NX4IO, I2C, RGB LEDs).
 * @return int XST_SUCCESS on success, XST_FAILURE on failure.
 */
int do_init(void) {
  int status = NX4IO_initialize(N4IO_BASEADDR);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  XIic_Config *ConfigPtr = XIic_LookupConfig(I2C_DEV_ID_ADDR);
  if (ConfigPtr == NULL) {
    return XST_FAILURE;
  }

  status = XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
  if (status != XST_SUCCESS) {
    return status;
  }

  status = XIic_Start(&IicInstance);
  if (status != XST_SUCCESS) {
    return status;
  }

  NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
  NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
  NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, min_duty);
  NX4IO_RGBLED_setDutyCycle(RGB2, min_duty, min_duty, min_duty);
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
void PID_Task(void *p) {
  sensor_Data *sensorData =
      (sensor_Data *)p;  // get the BME280 struct passed from main
  float luxOUT = 0;      // float percent value returned for LUX
  float tempOUT = 0;     // float percent value returned for temp
  float humOUT = 0;      // float percent value returned for hum
  uint8_t lightCtrl = 0; // 8-bit int value for controling "evironment lights"
  uint8_t tempCtrl = 0;  // 8-bit int value for controling "evironment temp"
  uint8_t fanCtrl =
      0; // 8-bit int value for controling fans, affected by humidty temp
  uint16_t btnSws; // value recieved from the input task Q
  uint8_t btns =
      0x10; // btn values parsed from btnSws, start with center button "pressed"
  uint8_t sws;            // switch values parsed from btnSws
  uint8_t incrScaler = 1; // scaler for changing increment size
  TickType_t lastLuxTick =
      xTaskGetTickCount(); // used for more accurate delta t values
  TickType_t lastBMETick =
      xTaskGetTickCount(); // used for sampling frequency of BME280
  static bool isInitialized =
      false; // true if the init function has run at least once

  // create PID structs for temperature, humidity, and lux control
  static PID_t pidLux, pidTemp, pidHum;

  // initialize the pid struct if it hasn't been
  if (!isInitialized) {
    isInitialized = (pid_init(&pidLux, 'L') && pid_init(&pidTemp, 'T') &&
                     pid_init(&pidHum, 'H'));
    if (!isInitialized) {
      xil_printf("PID struct initialization failed\r\n");
    }
  }

  // set initial data to PID set points to mitigate false on for PID control
  sensorData->humidity = pidHum.setpoint;
  sensorData->luminosity = pidLux.setpoint;
  sensorData->temperature = pidTemp.setpoint;

  // main task loop
  while (1) {
    // recieve message from input task, 16-bit uint that contains switch and
    // button values
    if (xQueueReceive(toPID, &btnSws, mainDONT_BLOCK) == pdPASS) {
      // parse values recieved from input task
      if (btnSws & 0x1300) // center, left, or right button pressed
      {
        btns = (btnSws & 0x1F00) >> 8;
      } else // get the new buttons presses and preserve last c/l/r press
      {
        btns = ((btnSws & 0x1F00) >> 8) | btns;
      }
      // get the new switch values
      sws = (btnSws & 0x0FF);
    } else {
      // btn values should retain the last btnc/bntl/btnr value that was pressed
      // sws values should retain the last switch values
      btns &= 0x13;
      sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // get lower 8 switches
      NX4IO_setLEDs(sws);
    }

    // updated increment scaler based on the switch 3 and 4
    if (sws & 0x08) {
      incrScaler = 5; // if sw 3 or 3 and 4 scale by 5x
    } else if (sws & 0x10) {
      incrScaler = 10; // if only sw 4 scale by 10x
    } else {
      incrScaler = 1; // if neither sw 3 or 4 scaler is 1x
    }

    // Get TSL2561 reading every 420ms and run PID algorithm for lux
    TickType_t currentLuxTick = xTaskGetTickCount();
    if (currentLuxTick - lastLuxTick >= 42) {
      float ch0 = tsl2561_readChannel(
          &IicInstance, TSL2561_CHANNEL_0); // visible and infrared
      float ch1 =
          tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1); // just infrared
      sensorData->luminosity =
          (uint16_t)ch0 - ((uint16_t)ch1 * 0.5); // calculate lux value

      // update delta_t for PID calculations
      pidLux.delta_t = ((currentLuxTick - lastLuxTick) * (1 / 100.0f));
      // update last tick time for use in next dt calculation
      lastLuxTick = currentLuxTick;

      // get correction percentage for lux
      luxOUT = pid_funct(&pidLux, (int32_t)sensorData->luminosity);
    }

    // Get BME280 reading when semaphore is available and run PID algorithm for
    // temp and humidity
    TickType_t currentBMETick = xTaskGetTickCount();
    if (xSemaphoreTake(bme280_sem,
                       mainDONT_BLOCK)) // get BME280 data aprox every 1 second
    {
      // get time between samples for temp and humidty
      pidHum.delta_t = ((currentBMETick - lastBMETick) * (1 / 100.0f));
      pidTemp.delta_t = ((currentBMETick - lastBMETick) * (1 / 100.0f));

      // update last tick time for use in next dt calculation
      lastBMETick = currentBMETick;

      // get humidty and temp correction percentages using values stored in
      // BME280 struct
      humOUT = pid_funct(&pidHum, ((int32_t)sensorData->humidity / 1024));
      tempOUT = pid_funct(&pidTemp, (sensorData->temperature / 100));

      xSemaphoreGive(bme280_sem); // release semaphore after reading
    }

    /*  Check which button was pressed and adjust PID values accordingly
     *   Center button pressed, increment humidity setpoint by 1
     *   Left button pressed, increment temperature setpoint by 1
     *   Right button pressed, increment lux setpoint by 1
     *   Diretion of increment dependent on up down buttons
     */
    switch (btns & 0x13) {
    case 0x10: // center button pressed, 7-seg display humidity
      displayHelper(&pidHum, btns, sensorData->humidity / 1024, incrScaler);
      break;
    case 0x02: // left button pressed, 7-seg display temp
      displayHelper(&pidTemp, btns, sensorData->temperature / 100, incrScaler);
      break;
    case 0x01: // right button pressed, 7-seg display lux
      displayHelper(&pidLux, btns, sensorData->luminosity, incrScaler);
      break;
    default:
      break;
    }

    /*PWM Write
     *   use the PID output to adjust the duty cycle and write it to the
     *       -heater
     *       -light
     *       -fan
     *   if the percentage returned is positive, the intensity needs to be
     * increased because the sensor is reading a value lower than the setpoint
     *   if it's negative the intensity needs to be decreased becasue the
     *   sensor is reading a value higher than the setpoint
     *   using NX4IO_RGBLED_setDutyCycle command
     */

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
    if ((((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
         ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
        (((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
         ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05)))) {
      NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, max_duty,
                                min_duty); // All green
    } else if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
                 ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
               !(((sensorData->temperature / 100) >=
                  (pidTemp.setpoint * 0.95)) &&
                 ((sensorData->temperature / 100) <=
                  (pidTemp.setpoint * 1.05)))) {
      NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, max_duty);
      fanCtrl =
          correctedSignal(fanCtrl, (humOUT + tempOUT) / 2,
                          true); // average humidity and temp for fan control
      tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
    } else {
      if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
            ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
          (((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
           ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05)))) {
        NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, max_duty);
        fanCtrl =
            correctedSignal(fanCtrl, (humOUT + tempOUT) / 2,
                            true); // average humidity and temp for fan control
      } else {
        NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, min_duty);
        fanCtrl =
            correctedSignal(fanCtrl, (humOUT + tempOUT) / 2,
                            true); // average humidity and temp for fan control
        tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
      }
    }
    lightCtrl = correctedSignal(lightCtrl, luxOUT, false);

    // check switchs 0-2 and determin what value to set fan on
    // fan will be driven by PID output or predefined percentages of
    // of 25, 50, and 75
    switch (sws & 0x07) {
    case 0x01:
      NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t)(0.25 * max_duty),
                                lightCtrl);
      break;
    case 0x02:
      NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t)(0.50 * max_duty),
                                lightCtrl);
      break;
    case 0x04:
      NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t)(0.75 * max_duty),
                                lightCtrl);
      break;
    default:
      NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, fanCtrl, lightCtrl);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(
        10)); // 100ms delay to allow time for interrupts to be processed
  }
}

/************************Display Task****************************************
 *   Gets lux and setpoint values from Q and updates 7-seg display
 *****************************************************************************/
void Display_Task(void *p) {
  uint32_t recievedLux;
  uint16_t setpnt = 0x0000, luxVal;
  while (1) {
    // recieve new sensor lux reading and setpoint values
    xQueueReceive(fromPID, &recievedLux, portMAX_DELAY);

    // make sure old values are cleared
    setpnt &= 0x0000;
    luxVal &= 0x0000;

    // parse values
    luxVal |= (recievedLux & lux_mask);
    setpnt |= ((recievedLux >> 16) & lux_mask);

    // write values to 7-seg display
    NX410_SSEG_setAllDigits(SSEGHI, (uint8_t)(setpnt / 100),
                            (uint8_t)((setpnt % 100) / 10),
                            (uint8_t)((setpnt % 100) % 10), CC_BLANK, DP_NONE);
    NX410_SSEG_setAllDigits(SSEGLO, (uint8_t)(luxVal / 100),
                            (uint8_t)((luxVal % 100) / 10),
                            (uint8_t)((luxVal % 100) % 10), CC_BLANK, DP_NONE);
  }
}

/*********************PID Initialization*************************************
 *   Initializing PID structure for use in the PI Task
 *****************************************************************************/
bool pid_init(PID_t *pid, char sensor) {
  pid->Kp = 1;
  pid->Ki = 0.02;
  pid->Kd = 0.001;
  if (sensor == 'L') {
    pid->setpoint = 200;
  } else if (sensor == 'H') {
    pid->setpoint = 35;
  } else {
    pid->setpoint = 21;
  }
  pid->integral = 0;
  pid->prev_error = 0;
  pid->delta_t = 1; // set to the worst case sampling time but will dynamically
                    // update in use

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
float pid_funct(PID_t *pid, int32_t lux_value) {
  // limits for integral range
  float max_int = 512;
  float min_int = -512;

  // e(t), error at time of sample
  float error = pid->setpoint - lux_value;

  // proportional
  float Pterm;
  // set to zero if switch[0] is O, affectively disabling proportional control
  Pterm = pid->Kp * error;

  // update integral, and get integral term
  // clamps to prevent Iterm from generating a value that would exceed 50% pwm
  // range
  pid->integral += (error * pid->delta_t);
  if (pid->integral >= max_int) {
    pid->integral = max_int;
  } else if (pid->integral <= min_int) {
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
  return ((!pid->prev_error) && (!error))
             ? (error / (float)pid->setpoint)
             : ((Pterm + Iterm + Dterm) / (float)pid->setpoint);
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
uint8_t correctedSignal(uint8_t enviro, float pidOut, bool fanCtrl) {
  float correctedPidOut = pidOut;
  if (fanCtrl) {
    correctedPidOut *= (-1);
  }

  if (((float)enviro + (correctedPidOut * max_duty)) > max_duty) {
    enviro = max_duty;
  } else if (((float)enviro + (correctedPidOut * max_duty)) < min_duty) {
    enviro = min_duty;
  } else {
    enviro = (uint8_t)(enviro + (correctedPidOut * max_duty));
  }

  return enviro;
}

/***********************7-Seg Display Helper********************************
 *   Create the message to send to the 7-Seg displae
 *
 *   @pid: Pointer to PID struct for desired sensor
 *   @btns: button value sent to PID task, used to determine if increment or
 *decrement
 *   @sensorVal: Reading from sensor
 *   @incr: Increment amount, this should be 1 * (scaling for sensor)
 *   lux sensor has scaling of 1
 *   temp sensor has scaling of 100
 *   humidty sensor has scaling of 1024*
 *****************************************************************************/
void displayHelper(PID_t *pid, uint8_t btns, uint16_t sensorVal,
                   uint16_t incr) {
  uint32_t message = 0x00000000; // make sure message is cleared before each use

  if (btns & 0x08) // up button pressed
  {
    pid->setpoint += incr;
  } else if (btns & 0x04) // down button pressed
  {
    pid->setpoint -= incr;
  }

  message |=
      ((sensorVal << 0) | ((pid->setpoint) << 16)); // build mesasge for 7-seg
  xQueueSend(fromPID, &message, mainDONT_BLOCK);    // send message to 7-seg
}

void cleanup_system(void) {
  // Stop the scheduler (optional, depending on use case)
  vTaskSuspendAll();

  // Delete tasks (requires task handles)
  if (xTaskGetHandle("Parse_Input") != NULL)
    vTaskDelete(xTaskGetHandle("Parse_Input"));
  if (xTaskGetHandle("PID") != NULL)
    vTaskDelete(xTaskGetHandle("PID"));
  if (xTaskGetHandle("Disp") != NULL)
    vTaskDelete(xTaskGetHandle("Disp"));
  if (xTaskGetHandle("BME280") != NULL)
    vTaskDelete(xTaskGetHandle("BME280"));
  if (xTaskGetHandle("OLED") != NULL)
    vTaskDelete(xTaskGetHandle("OLED"));
  if (xTaskGetHandle("I2C") != NULL)
    vTaskDelete(xTaskGetHandle("I2C"));

  // Delete queues
  if (i2c_request_queue != NULL) {
    vQueueDelete(i2c_request_queue);
    i2c_request_queue = NULL;
  }
  if (toPID != NULL) {
    vQueueDelete(toPID);
    toPID = NULL;
  }
  if (fromPID != NULL) {
    vQueueDelete(fromPID);
    fromPID = NULL;
  }

  // Delete semaphores
  if (i2c_sem != NULL) {
    vSemaphoreDelete(i2c_sem);
    i2c_sem = NULL;
  }
  if (binary_sem != NULL) {
    vSemaphoreDelete(binary_sem);
    binary_sem = NULL;
  }
  if (bme280_sem != NULL) {
    vSemaphoreDelete(bme280_sem);
    bme280_sem = NULL;
  }
  if (oled_sem != NULL) {
    vSemaphoreDelete(oled_sem);
    oled_sem = NULL;
  }

  // Stop I2C hardware
  XIic_Stop(&IicInstance);

  // Disable GPIO interrupts
  XGpio_InterruptDisable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
  XGpio_InterruptGlobalDisable(&xInputGPIOInstance);

  xil_printf("System cleanup completed\r\n");
}

void I2C_Task(void *p) {
  i2c_request_t req;

  while (1) {
    if (xQueueReceive(i2c_request_queue, &req, portMAX_DELAY) == pdPASS) {
      xSemaphoreTake(i2c_sem, portMAX_DELAY);

      switch (req.type) {
      case READ_TSL2561_CH0:
        *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
        break;

      case READ_TSL2561_CH1:
        *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
        break;

      case READ_BME280_TEMP:
      case READ_BME280_HUM:
      case READ_BME280_PRESS: {
        struct bme280_uncomp_data uncomp_data;
        uint8_t buffer[8];
        int status;

        buffer[0] = REG_DATA;
        status = XIic_Send(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1,
                           XIIC_REPEATED_START);
        status += XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8,
                            XIIC_STOP);

        if (status == 9) { // 1 byte sent + 8 bytes received
          uncomp_data.pressure = ((uint32_t)buffer[0] << 12) |
                                 ((uint32_t)buffer[1] << 4) | (buffer[2] >> 4);
          uncomp_data.temperature = ((uint32_t)buffer[3] << 12) |
                                    ((uint32_t)buffer[4] << 4) |
                                    (buffer[5] >> 4);
          uncomp_data.humidity = ((uint32_t)buffer[6] << 8) | buffer[7];

          if (req.type == READ_BME280_TEMP) {
            *req.result =
                (float)compensate_temperature(&uncomp_data, &calib_data);
          } else if (req.type == READ_BME280_HUM) {
            *req.result = (float)compensate_humidity(&uncomp_data, &calib_data);
          } else if (req.type == READ_BME280_PRESS) {
            *req.result = (float)compensate_pressure(&uncomp_data, &calib_data);
          }

          xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
        } else {
          *req.result = 0.0f;
          xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
          xil_printf("[ERROR] BME280 read failed in I2C_Task, status: %d\r\n",
                     status);
        }
        break;
      }
      case WRITE_LCD_CMD: {
        uint8_t buffer[2] = {OLED_CMD, req.cmd};
        XIic_Send(IicInstance.BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
        if (req.reply_queue) {
          uint8_t reply = 1;
          xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
        }
        break;
      }

      case WRITE_LCD_DATA: {
        uint8_t buffer[129]; // 1 control + 128 bytes
        buffer[0] = OLED_DATA;
        uint32_t bytes_to_send = (req.len > 128) ? 128 : req.len;
        memcpy(&buffer[1], req.data, bytes_to_send);
        XIic_Send(IicInstance.BaseAddress, OLED_I2C_ADDR, buffer,
                  bytes_to_send + 1, XIIC_STOP);
        if (req.reply_queue) {
          uint8_t reply = 1;
          xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
        }
        break;
      }

      default:
        xil_printf("[ERROR] Unknown I2C request type: %d\r\n", req.type);
        break;
      }

      xSemaphoreGive(i2c_sem); // Release I2C bus
    }
  }
}