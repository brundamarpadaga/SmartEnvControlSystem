/*
 * ===========================================================================
 * main.c - Entry point for TSL2561 and BME280-based PID control system
 * ===========================================================================
 * Purpose: Initializes the FreeRTOS environment, configures TSL2561 and BME280
 *          sensors, and creates tasks for environmental monitoring and control.
 *          Handles lux, temperature, pressure, and humidity readings, runs a
 *          PID controller to adjust actuators, and manages user input and
 *          displays.
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
// I2C instance used for communication with TSL2561, BME280, and OLED
XIic IicInstance;
// Structure to hold BME280 sensor calibration data read during initialization
struct bme280_calib_data calib_data;
// Semaphore to synchronize GPIO interrupt handling for button/switch inputs
SemaphoreHandle_t binary_sem = NULL;
// Semaphore to ensure exclusive access to BME280 sensor readings
SemaphoreHandle_t bme280_sem = NULL;
// Semaphore to protect OLED display access from multiple tasks
SemaphoreHandle_t oled_sem = NULL;
// Semaphore to manage access to the shared I2C bus
SemaphoreHandle_t i2c_sem = NULL;
// GPIO instance for reading button and switch states
XGpio xInputGPIOInstance;
// Queue to send button/switch data to the PID task
QueueHandle_t toPID = NULL;
// Queue to send PID output data to the display task
QueueHandle_t fromPID = NULL;
// Queue to handle I2C transaction requests from various tasks
QueueHandle_t i2c_request_queue = NULL;

/* ===========================================================================
 * Macro Definitions
 * ===========================================================================
 */

/**
 * @brief Logging macro for conditional debug output.
 * @details Prints messages via xil_printf if level <= DEBUG_LEVEL.
 *          Used throughout the code to log info, warnings, or errors.
 */
#define LOG(level, fmt, ...)                                                                       \
    if (level <= DEBUG_LEVEL)                                                                      \
    xil_printf(fmt, ##__VA_ARGS__)

/* ===========================================================================
 * Function Definitions
 * ===========================================================================
 */

/**
 * @brief Program entry point.
 * @details Initializes hardware, sensors, FreeRTOS primitives, and tasks.
 *          This is where the system boots up, sets up all components, and
 *          starts the FreeRTOS scheduler to run the tasks indefinitely.
 * @return int Never returns in FreeRTOS; -1 on initialization failure.
 */
int main(void)
{
    // Print startup message to indicate system is booting
    xil_printf("Hello from FreeRTOS LUX and BME280 PID Controller\r\n");

    // Set up GPIO and interrupt system for user inputs
    prvSetupHardware();

    // Initialize peripherals like NX4IO, I2C, and RGB LEDs
    if (do_init() != XST_SUCCESS)
    {
        xil_printf("[ERROR] Hardware initialization failed\r\n");
        cleanup_system();
        return -1; // Exit with error code if initialization fails
    }

    // Create binary semaphore for I2C bus access control
    i2c_sem = xSemaphoreCreateBinary();
    if (i2c_sem == NULL)
    {
        xil_printf("[ERROR] I2C semaphore creation failed\r\n");
        cleanup_system();
        return -1; // Exit if semaphore creation fails
    }
    xSemaphoreGive(i2c_sem); // Initially release the semaphore

    // Initialize the TSL2561 light sensor over I2C
    if (tsl2561_init(&IicInstance) != XST_SUCCESS)
    {
        xil_printf("[ERROR] TSL2561 initialization failed\r\n");
        cleanup_system();
        return -1; // Exit if sensor initialization fails
    }

    // Initialize BME280 sensor and read its calibration data
    if (bme_init(&IicInstance) != 0 || bme_read_calibration_data(&IicInstance, &calib_data) != 0)
    {
        xil_printf("[ERROR] BME280 initialization or calibration failed\r\n");
        cleanup_system();
        return -1; // Exit if BME280 setup fails
    }

    // Initialize the OLED display for showing sensor data
    if (lcd_init(&IicInstance) != XST_SUCCESS)
    {
        xil_printf("[ERROR] OLED initialization failed\r\n");
        cleanup_system();
        return -1; // Exit if OLED setup fails
    }

    // Create semaphore for GPIO interrupt synchronization
    binary_sem = xSemaphoreCreateBinary();
    if (binary_sem == NULL)
    {
        xil_printf("[ERROR] GPIO semaphore creation failed\r\n");
        cleanup_system();
        return -1; // Exit if semaphore creation fails
    }
    xSemaphoreGive(binary_sem); // Initially release the semaphore

    // Create semaphore for BME280 sensor access
    bme280_sem = xSemaphoreCreateBinary();
    if (bme280_sem == NULL)
    {
        xil_printf("[ERROR] BME280 semaphore creation failed\r\n");
        cleanup_system();
        return -1; // Exit if semaphore creation fails
    }
    xSemaphoreGive(bme280_sem); // Initially release the semaphore

    // Create semaphore for OLED display access
    oled_sem = xSemaphoreCreateBinary();
    if (oled_sem == NULL)
    {
        xil_printf("[ERROR] OLED semaphore creation failed\r\n");
        cleanup_system();
        return -1; // Exit if semaphore creation fails
    }
    xSemaphoreGive(oled_sem); // Initially release the semaphore

    // Create queues for inter-task communication
    i2c_request_queue = xQueueCreate(10, sizeof(i2c_request_t));              // I2C requests
    toPID             = xQueueCreate(mainQUEUE_LENGTH + 1, sizeof(uint16_t)); // Input to PID
    fromPID           = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));     // PID to display
    if (!i2c_request_queue || !toPID || !fromPID)
    {
        xil_printf("[ERROR] Queue creation failed\r\n");
        cleanup_system();
        return -1; // Exit if any queue creation fails
    }

    // Static structure to hold sensor data shared between tasks
    static sensor_Data sensorData; // Assuming sensor_Data is defined in pidtask.h

    // Create tasks for system operation
    xTaskCreate(Parse_Input_Task,
                "Parse_Input",
                configMINIMAL_STACK_SIZE,
                NULL,
                3,
                NULL); // Task to handle button/switch inputs
    xTaskCreate(PID_Task, "PID", configMINIMAL_STACK_SIZE, &sensorData, 1, NULL);
    // Task for PID control and actuator adjustment
    xTaskCreate(Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // Task to update 7-segment display
    xTaskCreate(BME280_Task,
                "BME280",
                configMINIMAL_STACK_SIZE * 2,
                &sensorData,
                1,
                NULL); // Task to read BME280 sensor
    xTaskCreate(LCD_Task,
                "OLED",
                configMINIMAL_STACK_SIZE * 4,
                &sensorData,
                2,
                NULL); // Task to update OLED display
    xTaskCreate(I2C_Task, "I2C", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    // Task to handle I2C transactions

    // Start the FreeRTOS scheduler to begin task execution
    xil_printf("Starting the scheduler\r\n");
    vTaskStartScheduler();

    // If scheduler exits (shouldn't happen), clean up and exit
    cleanup_system();
    return -1;
}

/**
 * @brief Configures GPIO and interrupt system.
 * @details Enables interrupts for both buttons and switches to detect user
 *          input. Sets up the GPIO directions and installs the interrupt handler.
 */
void prvSetupHardware(void)
{
    uint32_t            xStatus;              // Status variable for checking operation success
    const unsigned char ucSetToInput = 0xFFU; // Set all GPIO pins as inputs

    xil_printf("Initializing GPIO's\r\n"); // Log GPIO initialization start

    // Initialize GPIO instance for buttons and switches
    xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (xStatus == XST_SUCCESS)
    {
        // Install interrupt handler for GPIO interrupts
        xStatus = xPortInstallInterruptHandler(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL);
        if (xStatus == pdPASS)
        {
            xil_printf("Buttons and switches interrupt handler installed\r\n");
            // Set button channel as input
            XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, ucSetToInput);
            // Set switch channel as input
            XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, ucSetToInput);
            // Enable the interrupt in the interrupt controller
            vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);
            // Enable interrupts for both channels (buttons and switches)
            XGpio_InterruptEnable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
            // Globally enable GPIO interrupts
            XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
        }
    }
    // Assert if any step fails (for debugging)
    configASSERT(xStatus == pdPASS);
}

/**
 * @brief GPIO interrupt service routine.
 * @details Called when a button or switch state changes. Signals the input
 *          task via semaphore and clears the interrupt.
 * @param pvUnused Unused parameter required by ISR signature.
 */
void gpio_intr(void* pvUnused __attribute__((unused)))
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Flag for context switch

    // If semaphore exists, signal the input task
    if (binary_sem != NULL)
    {
        xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);
    }

    // Clear interrupts for both channels
    XGpio_InterruptClear(&xInputGPIOInstance, XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
    // Yield if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Task to process user inputs.
 * @details Reads button and switch states, debounces them, and sends combined
 *          data to the PID task via queue. Updates LEDs to reflect switch states.
 * @param p Unused parameter required by FreeRTOS task signature.
 */
void Parse_Input_Task(void* p __attribute__((unused)))
{
    uint8_t          btns = 0x00, last_btns = 0x00;        // Current and previous button states
    uint8_t          sws = 0x00, last_sws = 0x00;          // Current and previous switch states
    uint16_t         ValueToSend    = 0x0000;              // Combined button/switch data to send
    TickType_t       last_debounce  = xTaskGetTickCount(); // Last debounce timestamp
    const TickType_t debounce_delay = pdMS_TO_TICKS(20);   // 20ms debounce period

    while (1)
    { // Infinite task loop
        // Wait for GPIO interrupt with 500ms timeout
        if (xSemaphoreTake(binary_sem, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            TickType_t now = xTaskGetTickCount(); // Current time
            // Check if enough time has passed for debouncing
            if (now - last_debounce >= debounce_delay)
            {
                btns = NX4IO_getBtns() & BUTTON_MASK;                 // Read and mask button states
                sws  = (uint8_t) (NX4IO_getSwitches() & SWITCH_MASK); // Read and mask switches
                // If state has changed, update and send data
                if (btns != last_btns || sws != last_sws)
                {
                    ValueToSend = (btns << 8) | sws; // Combine buttons (high) and switches (low)
                    NX4IO_setLEDs(sws);              // Reflect switch states on LEDs
                    // Send data to PID task queue with timeout
                    if (xQueueSend(toPID, &ValueToSend, QUEUE_TIMEOUT_TICKS) != pdPASS)
                    {
                        LOG(LOG_LEVEL_WARNING, "[WARNING] Queue to PID full\r\n");
                    }
                    // Log the input change
                    LOG(LOG_LEVEL_INFO,
                        "[INFO] Input updated: btns=0x%02x, sws=0x%02x\r\n",
                        btns,
                        sws);
                    last_btns     = btns; // Update previous states
                    last_sws      = sws;
                    last_debounce = now; // Update debounce timestamp
                }
            }
        }
    }
}

/**
 * @brief Initializes peripherals (NX4IO, I2C, RGB LEDs).
 * @details Sets up the NX4IO board, configures I2C, and initializes RGB LEDs
 *          with minimum duty cycle.
 * @return int XST_SUCCESS on success, XST_FAILURE on failure.
 */
int do_init(void)
{
    // Initialize NX4IO board (buttons, switches, LEDs, etc.)
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE; // Return failure if NX4IO init fails
    }

    // Look up I2C configuration based on device ID
    XIic_Config* ConfigPtr = XIic_LookupConfig(I2C_DEV_ID_ADDR);
    if (ConfigPtr == NULL)
    {
        return XST_FAILURE; // Return failure if config lookup fails
    }

    // Initialize I2C instance with configuration
    status = XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
    if (status != XST_SUCCESS)
    {
        return status; // Return failure if I2C init fails
    }

    // Start the I2C peripheral
    status = XIic_Start(&IicInstance);
    if (status != XST_SUCCESS)
    {
        return status; // Return failure if I2C start fails
    }

    // Enable all channels (R, G, B) for both RGB LEDs
    NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
    NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
    // Set initial duty cycle to minimum for both RGB LEDs
    NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, min_duty);
    NX4IO_RGBLED_setDutyCycle(RGB2, min_duty, min_duty, min_duty);
    return XST_SUCCESS; // Return success if all steps complete
}

/**************************PID Task******************************************
 *   Task Handles the Following:
 *   - Reads parameter message from MsgQ (button/switch inputs)
 *   - Updates new control/setpoint parameters based on inputs
 *   - Gets current lux reading from TSL2561 sensor
 *   - Executes PID algorithm for lux, temperature, and humidity control
 *   - Drives PWM signals for LEDs (light, heater, fan) via RGB commands
 *   - Writes to display thread MsgQ to update setpoint and current values
 *****************************************************************************/
void PID_Task(void* p)
{
    // Pointer to shared sensor data structure passed from main
    sensor_Data* sensorData = (sensor_Data*) p;
    float        luxOUT     = 0;    // PID output percentage for lux control
    float        tempOUT    = 0;    // PID output percentage for temperature control
    float        humOUT     = 0;    // PID output percentage for humidity control
    uint8_t      lightCtrl  = 0;    // Current duty cycle for environment lights
    uint8_t      tempCtrl   = 0;    // Current duty cycle for heater
    uint8_t      fanCtrl    = 0;    // Current duty cycle for fan (affected by temp/humidity)
    uint16_t     btnSws;            // 16-bit value received from input task (buttons + switches)
    uint8_t      btns = 0x10;       // Button states, initialized with center button "pressed"
    uint8_t      sws;               // Switch states parsed from btnSws
    uint8_t      incrScaler    = 1; // Scaling factor for setpoint increments (1x, 5x, 10x)
    TickType_t   lastLuxTick   = xTaskGetTickCount(); // Timestamp of last lux reading
    TickType_t   lastBMETick   = xTaskGetTickCount(); // Timestamp of last BME280 reading
    static bool  isInitialized = false;               // Flag to track PID structure initialization
    // Static PID structures for each controlled parameter
    static PID_t pidLux, pidTemp, pidHum;

    // Initialize PID structures if not already done
    if (!isInitialized)
    {
        // Initialize PID controllers for lux, temperature, and humidity
        isInitialized =
            (pid_init(&pidLux, 'L') && pid_init(&pidTemp, 'T') && pid_init(&pidHum, 'H'));
        if (!isInitialized)
        {
            xil_printf("PID struct initialization failed\r\n");
        }
    }

    // Set initial sensor data to PID setpoints to avoid false control outputs
    sensorData->humidity    = pidHum.setpoint;
    sensorData->luminosity  = pidLux.setpoint;
    sensorData->temperature = pidTemp.setpoint;

    // Main task loop
    while (1)
    {
        // Check for new input data from Parse_Input_Task
        if (xQueueReceive(toPID, &btnSws, mainDONT_BLOCK) == pdPASS)
        {
            // Parse button states: preserve center/left/right if pressed
            if (btnSws & 0x1300) // Check for center, left, or right button
            {
                btns = (btnSws & 0x1F00) >> 8; // Extract button bits
            }
            else // Combine new button presses with preserved states
            {
                btns = ((btnSws & 0x1F00) >> 8) | btns;
            }
            sws = (btnSws & 0x0FF); // Extract switch bits
        }
        else
        {
            // Retain last center/left/right button state, update switches
            btns &= 0x13;
            sws = (uint8_t) (NX4IO_getSwitches() & 0x00FF); // Read current switches
            NX4IO_setLEDs(sws);                             // Update LEDs to match switch states
        }

        // Update increment scaler based on switches 3 and 4
        if (sws & 0x08)
        {
            incrScaler = 5; // Switch 3 on: 5x increment
        }
        else if (sws & 0x10)
        {
            incrScaler = 10; // Switch 4 on: 10x increment
        }
        else
        {
            incrScaler = 1; // No switches: 1x increment
        }

        // Read TSL2561 sensor every ~420ms and run PID for lux
        TickType_t currentLuxTick = xTaskGetTickCount();
        if (currentLuxTick - lastLuxTick >= 42)
        {
            // Read both channels of TSL2561 (visible+IR and IR only)
            float ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
            float ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
            // Calculate lux by subtracting half of IR from total
            sensorData->luminosity = (uint16_t) ch0 - ((uint16_t) ch1 * 0.5);
            // Calculate time delta for PID (in seconds)
            pidLux.delta_t = ((currentLuxTick - lastLuxTick) * (1 / 100.0f));
            lastLuxTick    = currentLuxTick; // Update timestamp
            // Compute PID correction for lux
            luxOUT = pid_funct(&pidLux, (int32_t) sensorData->luminosity);
        }

        // Read BME280 sensor when semaphore is available (~1s intervals)
        TickType_t currentBMETick = xTaskGetTickCount();
        if (xSemaphoreTake(bme280_sem, mainDONT_BLOCK))
        {
            // Calculate time deltas for temp and humidity PID
            pidHum.delta_t  = ((currentBMETick - lastBMETick) * (1 / 100.0f));
            pidTemp.delta_t = ((currentBMETick - lastBMETick) * (1 / 100.0f));
            lastBMETick     = currentBMETick; // Update timestamp
            // Compute PID corrections using stored BME280 readings
            humOUT  = pid_funct(&pidHum, ((int32_t) sensorData->humidity / 1024));
            tempOUT = pid_funct(&pidTemp, (sensorData->temperature / 100));
            xSemaphoreGive(bme280_sem); // Release semaphore
        }

        /* Adjust setpoints based on button presses:
         * - Center: Adjust humidity setpoint
         * - Left: Adjust temperature setpoint
         * - Right: Adjust lux setpoint
         * Direction depends on up/down buttons
         */
        switch (btns & 0x13)
        {
            case 0x10: // Center button: Update humidity setpoint and display
                displayHelper(&pidHum, btns, sensorData->humidity / 1024, incrScaler);
                break;
            case 0x02: // Left button: Update temperature setpoint and display
                displayHelper(&pidTemp, btns, sensorData->temperature / 100, incrScaler);
                break;
            case 0x01: // Right button: Update lux setpoint and display
                displayHelper(&pidLux, btns, sensorData->luminosity, incrScaler);
                break;
            default:
                break; // No action if no relevant button pressed
        }

        /* PWM Control:
         * - Uses PID outputs to adjust duty cycles for:
         *   - Heater (tempCtrl)
         *   - Lights (lightCtrl)
         *   - Fan (fanCtrl)
         * - Positive PID output: Increase intensity (below setpoint)
         * - Negative PID output: Decrease intensity (above setpoint)
         */

        /* Status LED and Actuator Control:
         * - RGB1: Status LEDs (Red/Green/Yellow)
         * - RGB2: Actuators (Heater/Fan/Lights)
         * - Checks if temp and humidity are within 5% of setpoints
         * - Updates status LEDs and actuators accordingly
         */
        if ((((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
             ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
            (((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
             ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05))))
        {
            // Both temp and humidity in range: Green status LED
            NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, max_duty, min_duty);
        }
        else if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
                   ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
                 !(((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
                   ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05))))
        {
            // Both out of range: Red + Yellow status LEDs
            NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, max_duty);
            fanCtrl  = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true); // Average for fan
            tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
        }
        else
        {
            if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
                  ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
                (((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
                 ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05))))
            {
                // Humidity out: Yellow status LED
                NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, max_duty);
                fanCtrl = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true);
            }
            else
            {
                // Temperature out: Red status LED
                NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, min_duty);
                fanCtrl  = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true);
                tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
            }
        }
        lightCtrl = correctedSignal(lightCtrl, luxOUT, false); // Update light control

        // Override fan control based on switches 0-2 (25%, 50%, 75%, or PID)
        switch (sws & 0x07)
        {
            case 0x01:
                NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t) (0.25 * max_duty), lightCtrl);
                break;
            case 0x02:
                NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t) (0.50 * max_duty), lightCtrl);
                break;
            case 0x04:
                NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, (uint8_t) (0.75 * max_duty), lightCtrl);
                break;
            default:
                NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, fanCtrl, lightCtrl); // Use PID value
                break;
        }

        // Delay to allow interrupt processing and prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/************************Display Task****************************************
 *   Gets lux and setpoint values from queue and updates 7-segment display.
 *   Runs continuously to keep display updated with latest values.
 *****************************************************************************/
void Display_Task(void* p)
{
    uint32_t recievedLux; // Received 32-bit value (setpoint + lux)
    uint16_t setpnt = 0x0000,
             luxVal = 0x0000; // Parsed setpoint and lux values, explicitly initialized
    while (1)
    {
        // Wait indefinitely for new data from PID task
        xQueueReceive(fromPID, &recievedLux, portMAX_DELAY);

        // Clear old values before parsing new ones
        setpnt &= 0x0000;
        luxVal &= 0x0000;

        // Extract setpoint (high 16 bits) and lux (low 16 bits)
        luxVal |= (recievedLux & lux_mask);
        setpnt |= ((recievedLux >> 16) & lux_mask);

        // Update 7-segment display: High digits for setpoint
        NX410_SSEG_setAllDigits(SSEGHI,
                                (uint8_t) (setpnt / 100),
                                (uint8_t) ((setpnt % 100) / 10),
                                (uint8_t) ((setpnt % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
        // Low digits for current lux value
        NX410_SSEG_setAllDigits(SSEGLO,
                                (uint8_t) (luxVal / 100),
                                (uint8_t) ((luxVal % 100) / 10),
                                (uint8_t) ((luxVal % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
    }
}

/*********************PID Initialization*************************************
 *   Initializes PID structure with default gains and setpoints for use in
 *   the PID Task. Different setpoints are set based on sensor type.
 *****************************************************************************/
bool pid_init(PID_t* pid, char sensor)
{
    pid->Kp = 1.0f;   // Proportional gain, explicitly float
    pid->Ki = 0.02f;  // Integral gain, explicitly float
    pid->Kd = 0.001f; // Derivative gain, explicitly float
    // Set initial setpoint based on sensor type
    if (sensor == 'L')
    {
        pid->setpoint = 200; // Lux setpoint
    }
    else if (sensor == 'H')
    {
        pid->setpoint = 35; // Humidity setpoint
    }
    else
    {
        pid->setpoint = 21; // Temperature setpoint
    }
    pid->integral   = 0; // Initial integral term
    pid->prev_error = 0; // Initial previous error
    pid->delta_t    = 1; // Initial time delta (updated dynamically)
    return true;         // Indicate successful initialization
}

/*********************PID Algorithm*************************************
 *   Implements PID control:
 *   - Takes PID structure and current sensor value
 *   - Computes P, I, D terms and returns a correction percentage
 *   - P = Kp * error
 *   - I = Ki * integral(error * dt)
 *   - D = Kd * (delta_error / dt)
 *   - Error = setpoint - measured value
 *   - Used to adjust PWM signals for actuators
 *****************************************************************************/
float pid_funct(PID_t* pid, int32_t lux_value)
{
    float max_int = 512;  // Maximum integral limit
    float min_int = -512; // Minimum integral limit

    float error = pid->setpoint - lux_value; // Calculate current error
    float Pterm = pid->Kp * error;           // Proportional term

    // Update integral with error over time, clamp to prevent windup
    pid->integral += (error * pid->delta_t);
    if (pid->integral >= max_int)
    {
        pid->integral = max_int;
    }
    else if (pid->integral <= min_int)
    {
        pid->integral = min_int;
    }
    float Iterm = pid->Ki * pid->integral; // Integral term

    // Calculate derivative term based on error change
    float Dterm     = pid->Kd * ((error - pid->prev_error) / pid->delta_t);
    pid->prev_error = error; // Store current error for next iteration

    // Return correction percentage relative to setpoint
    return ((!pid->prev_error) && (!error)) ? (error / (float) pid->setpoint)
                                            : ((Pterm + Iterm + Dterm) / (float) pid->setpoint);
}

/*********************PID Helper*************************************
 *   Adjusts actuator duty cycle based on PID correction percentage.
 *   Ensures duty cycle stays within valid range (min_duty to max_duty).
 *****************************************************************************/
uint8_t correctedSignal(uint8_t enviro, float pidOut, bool fanCtrl)
{
    float correctedPidOut = pidOut;
    if (fanCtrl)
    {
        correctedPidOut *= (-1); // Invert for fan control (opposite direction)
    }

    // Apply correction and clamp to valid PWM range
    if (((float) enviro + (correctedPidOut * max_duty)) > max_duty)
    {
        enviro = max_duty;
    }
    else if (((float) enviro + (correctedPidOut * max_duty)) < min_duty)
    {
        enviro = min_duty;
    }
    else
    {
        enviro = (uint8_t) (enviro + (correctedPidOut * max_duty));
    }
    return enviro; // Return updated duty cycle
}

/***********************7-Seg Display Helper********************************
 *   Prepares and sends setpoint and sensor value to 7-segment display task.
 *   Adjusts setpoint based on up/down buttons.
 *****************************************************************************/
void displayHelper(PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr)
{
    uint32_t message = 0x00000000; // Clear message buffer

    // Adjust setpoint based on up/down buttons
    if (btns & 0x08) // Up button
    {
        pid->setpoint += incr;
    }
    else if (btns & 0x04) // Down button
    {
        pid->setpoint -= incr;
    }

    // Combine sensor value and setpoint into 32-bit message
    message |= ((sensorVal << 0) | ((pid->setpoint) << 16));
    xQueueSend(fromPID, &message, mainDONT_BLOCK); // Send to display task
}

void cleanup_system(void)
{
    // Suspend all tasks to prevent further execution
    vTaskSuspendAll();

    // Delete all created tasks if they exist
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

    // Delete all queues if they exist
    if (i2c_request_queue != NULL)
    {
        vQueueDelete(i2c_request_queue);
        i2c_request_queue = NULL;
    }
    if (toPID != NULL)
    {
        vQueueDelete(toPID);
        toPID = NULL;
    }
    if (fromPID != NULL)
    {
        vQueueDelete(fromPID);
        fromPID = NULL;
    }

    // Delete all semaphores if they exist
    if (i2c_sem != NULL)
    {
        vSemaphoreDelete(i2c_sem);
        i2c_sem = NULL;
    }
    if (binary_sem != NULL)
    {
        vSemaphoreDelete(binary_sem);
        binary_sem = NULL;
    }
    if (bme280_sem != NULL)
    {
        vSemaphoreDelete(bme280_sem);
        bme280_sem = NULL;
    }
    if (oled_sem != NULL)
    {
        vSemaphoreDelete(oled_sem);
        oled_sem = NULL;
    }

    // Stop I2C hardware to free resources
    XIic_Stop(&IicInstance);

    // Disable GPIO interrupts to prevent stray signals
    XGpio_InterruptDisable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalDisable(&xInputGPIOInstance);

    xil_printf("System cleanup completed\r\n"); // Log cleanup completion
}

void I2C_Task(void* p)
{
    i2c_request_t req; // Structure to hold I2C request details

    while (1)
    { // Infinite task loop
        // Wait for an I2C request from the queue
        if (xQueueReceive(i2c_request_queue, &req, portMAX_DELAY) == pdPASS)
        {
            xSemaphoreTake(i2c_sem, portMAX_DELAY); // Acquire I2C bus access

            // Handle different types of I2C requests
            switch (req.type)
            {
                case READ_TSL2561_CH0: // Read TSL2561 channel 0 (visible + IR)
                    *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
                    xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK); // Send result
                    break;

                case READ_TSL2561_CH1: // Read TSL2561 channel 1 (IR only)
                    *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
                    xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK); // Send result
                    break;

                case READ_BME280_TEMP: // Read BME280 temperature
                case READ_BME280_HUM:  // Read BME280 humidity
                case READ_BME280_PRESS:
                {                                          // Read BME280 pressure
                    struct bme280_uncomp_data uncomp_data; // Uncompensated data structure
                    uint8_t                   buffer[8];   // Buffer for raw sensor data
                    int                       status;      // Status of I2C operations

                    buffer[0] = REG_DATA; // Register address to read from
                    // Send register address with repeated start
                    status = XIic_Send(
                        IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
                    // Receive 8 bytes of raw data
                    status +=
                        XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP);

                    if (status == 9)
                    { // Success: 1 byte sent + 8 bytes received
                        // Parse raw pressure data (20 bits)
                        uncomp_data.pressure = ((uint32_t) buffer[0] << 12) |
                                               ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
                        // Parse raw temperature data (20 bits)
                        uncomp_data.temperature = ((uint32_t) buffer[3] << 12) |
                                                  ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
                        // Parse raw humidity data (16 bits)
                        uncomp_data.humidity = ((uint32_t) buffer[6] << 8) | buffer[7];

                        // Compensate based on request type
                        if (req.type == READ_BME280_TEMP)
                        {
                            *req.result = (float) compensate_temperature(&uncomp_data, &calib_data);
                        }
                        else if (req.type == READ_BME280_HUM)
                        {
                            *req.result = (float) compensate_humidity(&uncomp_data, &calib_data);
                        }
                        else if (req.type == READ_BME280_PRESS)
                        {
                            *req.result = (float) compensate_pressure(&uncomp_data, &calib_data);
                        }
                        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK); // Send result
                    }
                    else
                    {
                        *req.result = 0.0f; // Return 0 on failure
                        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
                        xil_printf("[ERROR] BME280 read failed in I2C_Task, status: %d\r\n",
                                   status);
                    }
                    break;
                }
                case WRITE_LCD_CMD:
                {                                            // Write command to OLED display
                    uint8_t buffer[2] = {OLED_CMD, req.cmd}; // Command buffer
                    XIic_Send(IicInstance.BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
                    if (req.reply_queue)
                    { // Send acknowledgment if queue provided
                        uint8_t reply = 1;
                        xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
                    }
                    break;
                }

                case WRITE_LCD_DATA:
                {                        // Write data to OLED display
                    uint8_t buffer[129]; // Buffer: 1 control byte + up to 128 data bytes
                    buffer[0]              = OLED_DATA;                       // Control byte
                    uint32_t bytes_to_send = (req.len > 128) ? 128 : req.len; // Cap at 128
                    memcpy(&buffer[1], req.data, bytes_to_send);              // Copy data to buffer
                    XIic_Send(IicInstance.BaseAddress,
                              OLED_I2C_ADDR,
                              buffer,
                              bytes_to_send + 1,
                              XIIC_STOP); // Send over I2C
                    if (req.reply_queue)
                    { // Send acknowledgment if queue provided
                        uint8_t reply = 1;
                        xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
                    }
                    break;
                }

                default: // Unknown request type
                    xil_printf("[ERROR] Unknown I2C request type: %d\r\n", req.type);
                    break;
            }

            xSemaphoreGive(i2c_sem); // Release I2C bus access
        }
    }
}