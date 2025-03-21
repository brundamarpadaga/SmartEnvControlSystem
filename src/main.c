/*
 * ===========================================================================
 * main.c - Entry point for TSL2561 and BME280-based PID Control System
 * ===========================================================================
 * Purpose: Serves as the main entry point for initializing and running a
 *          FreeRTOS-based environmental control system. Configures TSL2561
 *          (light) and BME280 (temperature, pressure, humidity) sensors via
 *          I2C, sets up tasks for monitoring and control, and manages real-time
 *          PID adjustments for actuators (fans, heaters, lights). Integrates
 *          user inputs from buttons/switches and displays data on an OLED and
 *          7-segment display.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 */

/* Include necessary headers */
#include "main.h" // Project-specific header with all required definitions

/* ===========================================================================
 * Global Variable Definitions
 * ===========================================================================
 */
/* I2C instance for communication with TSL2561 (light sensor), BME280 (environmental
   sensor), and SSD1306 OLED display. Initialized in do_init() and used across tasks. */
XIic IicInstance;

/* Calibration data structure for BME280, populated during initialization with
   coefficients read from sensor registers. Used for compensating raw sensor data. */
struct bme280_calib_data calib_data;

/* Binary semaphore for synchronizing GPIO interrupts from button/switch inputs.
   Ensures the Parse_Input_Task reacts only when an interrupt occurs. */
SemaphoreHandle_t binary_sem = NULL;

/* Binary semaphore for exclusive access to BME280 sensor readings, preventing
   concurrent access by multiple tasks (e.g., BME280_Task and PID_Task). */
SemaphoreHandle_t bme280_sem = NULL;

/* Binary semaphore to protect OLED display access, ensuring only one task (e.g.,
   LCD_Task) modifies the display at a time. */
SemaphoreHandle_t oled_sem = NULL;

/* Binary semaphore for managing shared I2C bus access across all I2C-related tasks.
   Critical for thread safety in a multi-task environment. */
SemaphoreHandle_t i2c_sem = NULL;

/* GPIO instance for interfacing with buttons and switches on the Nexys A7 board.
   Configured in prvSetupHardware() to handle user inputs. */
XGpio xInputGPIOInstance;

/* Queue for sending button and switch data from Parse_Input_Task to PID_Task.
   Sized to hold minimal data (mainQUEUE_LENGTH + 1) for efficiency. */
QueueHandle_t toPID = NULL;

/* Queue for sending PID output (setpoints and sensor values) from PID_Task to
   Display_Task for the 7-segment display. */
QueueHandle_t fromPID = NULL;

/* Queue for handling I2C transaction requests from tasks like BME280_Task,
   LCD_Task, and PID_Task. Sized for 10 requests to balance responsiveness and memory use. */
QueueHandle_t i2c_request_queue = NULL;

/* ===========================================================================
 * Macro Definitions
 * ===========================================================================
 */

/**
 * @brief Logging macro for conditional debug output.
 * @details Checks if the specified log level is at or below DEBUG_LEVEL, then
 *          prints the formatted message using xil_printf. Used for debugging
 *          with levels: ERROR (1), WARNING (2), INFO (3). Helps track system
 *          state without flooding output.
 * @param level Logging level to compare against DEBUG_LEVEL
 * @param fmt Format string for xil_printf
 * @param ... Variable arguments for formatting
 */
#define LOG(level, fmt, ...)                                                                       \
    do                                                                                             \
    {                                                                                              \
        if (level <= DEBUG_LEVEL)                                                                  \
            xil_printf(fmt, ##__VA_ARGS__);                                                        \
    } while (0)

/* ===========================================================================
 * Function Definitions
 * ===========================================================================
 */

/**
 * @brief Program entry point for the environmental control system.
 * @details Initializes hardware (GPIO, I2C, NX4IO), sensors (TSL2561, BME280),
 *          OLED display, FreeRTOS primitives (semaphores, queues), and tasks.
 *          Starts the scheduler to run tasks indefinitely. If any step fails,
 *          it cleans up and exits with an error code.
 * @return int Returns -1 on failure; never returns in normal FreeRTOS operation
 *          as the scheduler takes over.
 */
int main(void)
{
    // Print initial message to signal system startup
    xil_printf("Hello from FreeRTOS LUX and BME280 PID Controller\r\n");

    // Configure GPIO and interrupt system for buttons and switches
    prvSetupHardware();

    // Initialize hardware peripherals: NX4IO (buttons, LEDs), I2C, and RGB LEDs
    if (do_init() != XST_SUCCESS)
    {
        xil_printf("[ERROR] Hardware initialization failed\r\n");
        cleanup_system(); // Free resources on failure
        return -1;        // Exit with error
    }

    // Create I2C semaphore to manage bus access across tasks
    i2c_sem = xSemaphoreCreateBinary();
    if (i2c_sem == NULL)
    {
        xil_printf("[ERROR] I2C semaphore creation failed\r\n");
        cleanup_system();
        return -1;
    }
    xSemaphoreGive(i2c_sem); // Release semaphore initially to allow access

    // Initialize TSL2561 light sensor, setting up I2C communication and timing
    if (tsl2561_init(&IicInstance) != XST_SUCCESS)
    {
        xil_printf("[ERROR] TSL2561 initialization failed\r\n");
        cleanup_system();
        return -1;
    }

    // Initialize BME280 sensor and read its calibration data into calib_data
    if (bme_init(&IicInstance) != 0 || bme_read_calibration_data(&IicInstance, &calib_data) != 0)
    {
        xil_printf("[ERROR] BME280 initialization or calibration failed\r\n");
        cleanup_system();
        return -1;
    }

    // Initialize SSD1306 OLED display for showing sensor readings
    if (lcd_init(&IicInstance) != XST_SUCCESS)
    {
        xil_printf("[ERROR] OLED initialization failed\r\n");
        cleanup_system();
        return -1;
    }

    // Create semaphore for GPIO interrupt synchronization (button/switch inputs)
    binary_sem = xSemaphoreCreateBinary();
    if (binary_sem == NULL)
    {
        xil_printf("[ERROR] GPIO semaphore creation failed\r\n");
        cleanup_system();
        return -1;
    }
    xSemaphoreGive(binary_sem); // Release semaphore initially

    // Create semaphore for BME280 sensor access control
    bme280_sem = xSemaphoreCreateBinary();
    if (bme280_sem == NULL)
    {
        xil_printf("[ERROR] BME280 semaphore creation failed\r\n");
        cleanup_system();
        return -1;
    }
    xSemaphoreGive(bme280_sem); // Release semaphore initially

    // Create semaphore for OLED display access control
    oled_sem = xSemaphoreCreateBinary();
    if (oled_sem == NULL)
    {
        xil_printf("[ERROR] OLED semaphore creation failed\r\n");
        cleanup_system();
        return -1;
    }
    xSemaphoreGive(oled_sem); // Release semaphore initially

    // Create queues for task communication
    i2c_request_queue = xQueueCreate(10, sizeof(i2c_request_t)); // Queue for I2C requests
    toPID             = xQueueCreate(mainQUEUE_LENGTH + 1, sizeof(uint16_t)); // Queue for PID input
    fromPID           = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t)); // Queue for PID output
    if (!i2c_request_queue || !toPID || !fromPID)
    {
        xil_printf("[ERROR] Queue creation failed\r\n");
        cleanup_system();
        return -1; // Exit if any queue fails to create
    }

    // Define static sensor data structure shared across tasks (e.g., BME280_Task, PID_Task)
    static sensor_Data sensorData; // Holds temperature, humidity, pressure, and lux

    // Create FreeRTOS tasks with specific priorities and stack sizes
    xTaskCreate(Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    // Task to process button/switch inputs, priority 3
    xTaskCreate(PID_Task, "PID", configMINIMAL_STACK_SIZE, &sensorData, 1, NULL);
    // Task for PID control, priority 1, uses sensorData
    xTaskCreate(Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // Task for 7-segment display updates, priority 1
    xTaskCreate(BME280_Task, "BME280", configMINIMAL_STACK_SIZE * 2, &sensorData, 1, NULL);
    // Task to read BME280 sensor, priority 1, larger stack for data handling
    xTaskCreate(LCD_Task, "OLED", configMINIMAL_STACK_SIZE * 4, &sensorData, 2, NULL);
    // Task to update OLED display, priority 2, larger stack for string formatting
    xTaskCreate(I2C_Task, "I2C", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    // Task to handle I2C transactions, priority 2

    // Start FreeRTOS scheduler to begin task execution
    xil_printf("Starting the scheduler\r\n");
    vTaskStartScheduler();

    // If scheduler exits (unexpected), clean up resources and return error
    cleanup_system();
    return -1;
}

/**
 * @brief Configures GPIO hardware and interrupt system for user inputs.
 * @details Sets up GPIO channels for buttons (channel 1) and switches (channel 2)
 *          as inputs, installs an interrupt handler for state changes, and enables
 *          interrupts. Ensures reliable detection of user interactions.
 */
void prvSetupHardware(void)
{
    uint32_t            xStatus;              // Status variable to track operation success
    const unsigned char ucSetToInput = 0xFFU; // All 8 bits as inputs (0xFF = all 1s)

    xil_printf("Initializing GPIO's\r\n"); // Log start of GPIO setup

    // Initialize GPIO instance with device ID from xparameters.h
    xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (xStatus == XST_SUCCESS)
    {
        // Install interrupt handler for GPIO interrupts, linking to gpio_intr
        xStatus = xPortInstallInterruptHandler(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpio_intr, NULL);
        if (xStatus == pdPASS)
        {
            xil_printf("Buttons and switches interrupt handler installed\r\n");
            // Set GPIO channel 1 (buttons) as input
            XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, ucSetToInput);
            // Set GPIO channel 2 (switches) as input
            XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, ucSetToInput);
            // Enable interrupt in the Microblaze interrupt controller
            vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);
            // Enable interrupts for both channels (buttons and switches)
            XGpio_InterruptEnable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
            // Enable GPIO interrupts globally
            XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
        }
    }
    // Assert if initialization or handler setup fails (debugging check)
    configASSERT(xStatus == pdPASS);
}

/**
 * @brief GPIO interrupt service routine (ISR).
 * @details Triggered on button or switch state changes. Signals Parse_Input_Task
 *          via binary_sem to process the input and clears the interrupt flags.
 *          Runs in ISR context, so it’s kept minimal and fast.
 * @param pvUnused Unused parameter required by ISR signature.
 */
void gpio_intr(void* pvUnused __attribute__((unused)))
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Flag to trigger context switch if needed

    // Check if semaphore exists, then signal Parse_Input_Task to wake up
    if (binary_sem != NULL)
    {
        xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);
    }

    // Clear interrupt flags for both channels to acknowledge and reset
    XGpio_InterruptClear(&xInputGPIOInstance, XGPIO_IR_CH1_MASK | XGPIO_IR_CH2_MASK);
    // Yield to higher-priority tasks if woken (e.g., Parse_Input_Task)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Task to process user inputs from buttons and switches.
 * @details Waits for GPIO interrupts, reads button/switch states, debounces them
 *          (20ms delay), combines the data into a 16-bit value, and sends it to
 *          PID_Task via toPID queue. Updates LEDs to reflect switch states and
 *          logs changes for debugging.
 * @param p Unused parameter required by FreeRTOS task signature.
 */
void Parse_Input_Task(void* p __attribute__((unused)))
{
    uint8_t          btns = 0x00, last_btns = 0x00; // Current and last button states (5 bits)
    uint8_t          sws = 0x00, last_sws = 0x00;   // Current and last switch states (8 bits)
    uint16_t         ValueToSend    = 0x0000;       // 16-bit combined button/switch data
    TickType_t       last_debounce  = xTaskGetTickCount(); // Timestamp of last debounce check
    const TickType_t debounce_delay = pdMS_TO_TICKS(20);   // 20ms debounce period

    while (1) // Infinite loop to continuously monitor inputs
    {
        // Wait for GPIO interrupt signal with 500ms timeout
        if (xSemaphoreTake(binary_sem, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            TickType_t now = xTaskGetTickCount(); // Get current time for debouncing
            // Only process if enough time has passed since last change (debouncing)
            if (now - last_debounce >= debounce_delay)
            {
                btns = NX4IO_getBtns() & BUTTON_MASK; // Read buttons, mask to 5 bits
                sws =
                    (uint8_t) (NX4IO_getSwitches() & SWITCH_MASK); // Read switches, mask to 8 bits
                // Check if buttons or switches changed since last read
                if (btns != last_btns || sws != last_sws)
                {
                    ValueToSend =
                        (btns << 8) | sws; // Combine: buttons in high byte, switches in low
                    NX4IO_setLEDs(sws);    // Mirror switch states to LEDs for visual feedback
                    // Send combined data to PID task with a timeout
                    if (xQueueSend(toPID, &ValueToSend, QUEUE_TIMEOUT_TICKS) != pdPASS)
                    {
                        LOG(LOG_LEVEL_WARNING, "[WARNING] Queue to PID full\r\n");
                    }
                    // Log input changes at INFO level for debugging
                    LOG(LOG_LEVEL_INFO,
                        "[INFO] Input updated: btns=0x%02x, sws=0x%02x\r\n",
                        btns,
                        sws);
                    last_btns     = btns; // Update last states for next comparison
                    last_sws      = sws;
                    last_debounce = now; // Reset debounce timer
                }
            }
        }
    }
}

/**
 * @brief Initializes hardware peripherals for the system.
 * @details Configures NX4IO (buttons, switches, LEDs), initializes the I2C
 *          controller, and sets RGB LEDs to minimum duty cycle. Ensures all
 *          hardware is ready before tasks start.
 * @return int XST_SUCCESS if all steps succeed, XST_FAILURE if any step fails.
 */
int do_init(void)
{
    // Initialize NX4IO peripheral with its base address
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE; // Return failure if NX4IO setup fails
    }

    // Retrieve I2C configuration from device ID defined in xparameters.h
    XIic_Config* ConfigPtr = XIic_LookupConfig(I2C_DEV_ID_ADDR);
    if (ConfigPtr == NULL)
    {
        return XST_FAILURE; // Return failure if config lookup fails
    }

    // Initialize I2C instance with base address from config
    status = XIic_CfgInitialize(&IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
    if (status != XST_SUCCESS)
    {
        return status; // Return failure if I2C initialization fails
    }

    // Start I2C peripheral to enable communication
    status = XIic_Start(&IicInstance);
    if (status != XST_SUCCESS)
    {
        return status; // Return failure if I2C start fails
    }

    // Enable all RGB channels (R, G, B) for both RGB LED sets (RGB1, RGB2)
    NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
    NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
    // Set initial PWM duty cycle to minimum (0) for both RGB LEDs
    NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, min_duty);
    NX4IO_RGBLED_setDutyCycle(RGB2, min_duty, min_duty, min_duty);
    return XST_SUCCESS; // Return success if all steps complete
}

/**
 * @brief PID control task for environmental regulation.
 * @details Manages sensor readings (lux from TSL2561, temp/humidity from BME280),
 *          updates PID setpoints based on user inputs, runs PID algorithms, and
 *          adjusts actuator PWM signals (lights, heater, fan). Sends display data
 *          to Display_Task. Runs continuously with a 10ms delay to balance responsiveness.
 * @param p Pointer to shared sensor_Data structure.
 */
void PID_Task(void* p)
{
    sensor_Data* sensorData = (sensor_Data*) p; // Cast parameter to shared sensor data
    float        luxOUT     = 0;                // PID output for lux control (percentage)
    float        tempOUT    = 0;                // PID output for temperature control (percentage)
    float        humOUT     = 0;                // PID output for humidity control (percentage)
    uint8_t      lightCtrl  = 0;                // Current PWM duty cycle for environmental lights
    uint8_t      tempCtrl   = 0;                // Current PWM duty cycle for heater
    uint8_t      fanCtrl    = 0;    // Current PWM duty cycle for fan (temp/humidity influenced)
    uint16_t     btnSws;            // 16-bit input data from Parse_Input_Task (buttons + switches)
    uint8_t      btns = 0x10;       // Button states, default with center button "pressed"
    uint8_t      sws;               // Switch states extracted from btnSws
    uint8_t      incrScaler    = 1; // Scaling factor for setpoint adjustments (1x, 5x, 10x)
    TickType_t   lastLuxTick   = xTaskGetTickCount(); // Timestamp of last lux reading
    TickType_t   lastBMETick   = xTaskGetTickCount(); // Timestamp of last BME280 reading
    static bool  isInitialized = false;               // Flag to ensure PID structs initialize once
    static PID_t pidLux, pidTemp, pidHum;             // Static PID structs for lux, temp, humidity

    // Initialize PID controllers if not already done (runs once)
    if (!isInitialized)
    {
        isInitialized =
            (pid_init(&pidLux, 'L') && pid_init(&pidTemp, 'T') && pid_init(&pidHum, 'H'));
        if (!isInitialized)
        {
            xil_printf("PID struct initialization failed\r\n");
        }
    }

    // Set initial sensor values to PID setpoints to avoid erratic startup behavior
    sensorData->humidity    = pidHum.setpoint;
    sensorData->luminosity  = pidLux.setpoint;
    sensorData->temperature = pidTemp.setpoint;

    while (1) // Infinite loop for continuous PID control
    {
        // Check for new button/switch data from toPID queue (non-blocking)
        if (xQueueReceive(toPID, &btnSws, mainDONT_BLOCK) == pdPASS)
        {
            // If center, left, or right button pressed, update btns from input
            if (btnSws & 0x1300)
            {
                btns = (btnSws & 0x1F00) >> 8; // Extract 5 button bits
            }
            else
            {
                btns = ((btnSws & 0x1F00) >> 8) | btns; // Preserve existing states
            }
            sws = (btnSws & 0x0FF); // Extract 8 switch bits
        }
        else
        {
            // No new input: keep center/left/right, update switches directly
            btns &= 0x13;
            sws = (uint8_t) (NX4IO_getSwitches() & 0x00FF);
            NX4IO_setLEDs(sws); // Update LEDs to match switch states
        }

        // Adjust setpoint increment scaler based on switches 3 and 4
        if (sws & 0x08)
            incrScaler = 5; // 5x increment
        else if (sws & 0x10)
            incrScaler = 10; // 10x increment
        else
            incrScaler = 1; // 1x increment

        // Read TSL2561 every ~420ms and compute PID for lux
        TickType_t currentLuxTick = xTaskGetTickCount();
        if (currentLuxTick - lastLuxTick >= 42)
        {
            float ch0              = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
            float ch1              = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
            sensorData->luminosity = (uint16_t) ch0 - ((uint16_t) ch1 * 0.5);
            pidLux.delta_t         = ((currentLuxTick - lastLuxTick) * (1 / 100.0f));
            lastLuxTick            = currentLuxTick;
            luxOUT                 = pid_funct(&pidLux, (int32_t) sensorData->luminosity);
        }

        // Read BME280 data when available (non-blocking semaphore check)
        TickType_t currentBMETick = xTaskGetTickCount();
        if (xSemaphoreTake(bme280_sem, mainDONT_BLOCK))
        {
            pidHum.delta_t  = ((currentBMETick - lastBMETick) * (1 / 100.0f));
            pidTemp.delta_t = ((currentBMETick - lastBMETick) * (1 / 100.0f));
            lastBMETick     = currentBMETick;
            humOUT          = pid_funct(&pidHum, ((int32_t) sensorData->humidity / 1024));
            tempOUT         = pid_funct(&pidTemp, (sensorData->temperature / 100));
            xSemaphoreGive(bme280_sem);
        }

        // Adjust PID setpoints based on button presses
        switch (btns & 0x13)
        {
            case 0x10:
                displayHelper(&pidHum, btns, sensorData->humidity / 1024, incrScaler);
                break;
            case 0x02:
                displayHelper(&pidTemp, btns, sensorData->temperature / 100, incrScaler);
                break;
            case 0x01:
                displayHelper(&pidLux, btns, sensorData->luminosity, incrScaler);
                break;
            default:
                break;
        }

        // Update status LEDs and actuators based on PID outputs and setpoints
        if ((((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
             ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
            (((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
             ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05))))
        {
            NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, max_duty, min_duty); // Green LED
        }
        else if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
                   ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))) &&
                 !(((sensorData->temperature / 100) >= (pidTemp.setpoint * 0.95)) &&
                   ((sensorData->temperature / 100) <= (pidTemp.setpoint * 1.05))))
        {
            NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, max_duty); // Red + Yellow
            fanCtrl  = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true);
            tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
        }
        else
        {
            if (!(((sensorData->humidity / 1024) >= (pidHum.setpoint * 0.95)) &&
                  ((sensorData->humidity / 1024) <= (pidHum.setpoint * 1.05))))
            {
                NX4IO_RGBLED_setDutyCycle(RGB1, min_duty, min_duty, max_duty); // Yellow
                fanCtrl = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true);
            }
            else
            {
                NX4IO_RGBLED_setDutyCycle(RGB1, max_duty, min_duty, min_duty); // Red
                fanCtrl  = correctedSignal(fanCtrl, (humOUT + tempOUT) / 2, true);
                tempCtrl = correctedSignal(tempCtrl, tempOUT, false);
            }
        }
        lightCtrl = correctedSignal(lightCtrl, luxOUT, false);

        // Override fan PWM based on switch settings (0-2)
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
                NX4IO_RGBLED_setDutyCycle(RGB2, tempCtrl, fanCtrl, lightCtrl);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay to prevent task starvation
    }
}

/**
 * @brief Task to update the 7-segment display with PID data.
 * @details Receives setpoint and lux values from fromPID queue, parses them,
 *          and updates the 7-segment display (high digits for setpoint, low
 *          for lux). Runs continuously with blocking queue receive.
 * @param p Unused parameter for task signature.
 */
void Display_Task(void* p)
{
    uint32_t recievedLux;                      // 32-bit value from PID task (setpoint + lux)
    uint16_t setpnt = 0x0000, luxVal = 0x0000; // Parsed setpoint and lux values

    while (1) // Infinite loop for continuous display updates
    {
        // Wait indefinitely for data from PID task
        xQueueReceive(fromPID, &recievedLux, portMAX_DELAY);
        setpnt &= 0x0000;                           // Clear previous setpoint value
        luxVal &= 0x0000;                           // Clear previous lux value
        luxVal |= (recievedLux & lux_mask);         // Extract low 16 bits (lux)
        setpnt |= ((recievedLux >> 16) & lux_mask); // Extract high 16 bits (setpoint)
        // Update high digits (setpoint) on 7-segment display
        NX410_SSEG_setAllDigits(SSEGHI,
                                (uint8_t) (setpnt / 100),
                                (uint8_t) ((setpnt % 100) / 10),
                                (uint8_t) ((setpnt % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
        // Update low digits (lux) on 7-segment display
        NX410_SSEG_setAllDigits(SSEGLO,
                                (uint8_t) (luxVal / 100),
                                (uint8_t) ((luxVal % 100) / 10),
                                (uint8_t) ((luxVal % 100) % 10),
                                CC_BLANK,
                                DP_NONE);
    }
}

/**
 * @brief Initializes a PID controller structure with default parameters.
 * @details Sets gains (Kp, Ki, Kd) and initial setpoint based on sensor type
 *          ('L' for lux, 'T' for temp, 'H' for humidity). Used by PID_Task.
 * @param pid Pointer to PID structure to initialize.
 * @param sensor Sensor type identifier ('L', 'T', 'H').
 * @return bool Always returns true (successful initialization).
 */
bool pid_init(PID_t* pid, char sensor)
{
    pid->Kp = 1.0f;   // Proportional gain as float
    pid->Ki = 0.02f;  // Integral gain as float
    pid->Kd = 0.001f; // Derivative gain as float
    if (sensor == 'L')
        pid->setpoint = 200; // Lux default setpoint
    else if (sensor == 'H')
        pid->setpoint = 35; // Humidity default setpoint
    else
        pid->setpoint = 21; // Temperature default setpoint
    pid->integral   = 0;    // Reset integral term
    pid->prev_error = 0;    // Reset previous error
    pid->delta_t    = 1;    // Initial time delta (updated in PID_Task)
    return true;            // Indicate success
}

/**
 * @brief Computes PID correction for a sensor value.
 * @details Calculates proportional (P), integral (I), and derivative (D) terms
 *          based on setpoint and current value, returning a correction percentage.
 *          Clamps integral to prevent windup. Used by PID_Task for actuator control.
 * @param pid Pointer to PID structure with gains and state.
 * @param lux_value Current sensor reading (cast to int32_t).
 * @return float Correction percentage relative to setpoint.
 */
float pid_funct(PID_t* pid, int32_t lux_value)
{
    float max_int = 512;                       // Upper limit for integral term
    float min_int = -512;                      // Lower limit for integral term
    float error   = pid->setpoint - lux_value; // Compute current error
    float Pterm   = pid->Kp * error;           // Proportional term
    pid->integral += (error * pid->delta_t);   // Update integral term
    if (pid->integral >= max_int)              // Clamp integral to avoid windup
        pid->integral = max_int;
    else if (pid->integral <= min_int)
        pid->integral = min_int;
    float Iterm     = pid->Ki * pid->integral;                              // Integral term
    float Dterm     = pid->Kd * ((error - pid->prev_error) / pid->delta_t); // Derivative term
    pid->prev_error = error; // Save error for next iteration
    // Return correction as percentage; special case for zero error
    return ((!pid->prev_error) && (!error)) ? (error / (float) pid->setpoint)
                                            : ((Pterm + Iterm + Dterm) / (float) pid->setpoint);
}

/**
 * @brief Adjusts actuator PWM duty cycle based on PID output.
 * @details Applies PID correction to current duty cycle, inverting for fan control,
 *          and clamps result within min_duty and max_duty. Used by PID_Task.
 * @param enviro Current duty cycle value.
 * @param pidOut PID correction percentage.
 * @param fanCtrl True if controlling fan (inverts correction).
 * @return uint8_t Updated duty cycle.
 */
uint8_t correctedSignal(uint8_t enviro, float pidOut, bool fanCtrl)
{
    float correctedPidOut = pidOut; // Base correction value
    if (fanCtrl)
        correctedPidOut *= (-1); // Invert for fan (opposite response)
    // Apply correction and clamp to PWM range
    if (((float) enviro + (correctedPidOut * max_duty)) > max_duty)
        enviro = max_duty;
    else if (((float) enviro + (correctedPidOut * max_duty)) < min_duty)
        enviro = min_duty;
    else
        enviro = (uint8_t) (enviro + (correctedPidOut * max_duty));
    return enviro; // Return adjusted duty cycle
}

/**
 * @brief Helper function to update 7-segment display with PID data.
 * @details Adjusts setpoint based on up/down buttons and sends setpoint/sensor
 *          value pair to Display_Task via fromPID queue.
 * @param pid Pointer to PID structure to adjust.
 * @param btns Current button states.
 * @param sensorVal Current sensor reading.
 * @param incr Increment/decrement value for setpoint adjustment.
 */
void displayHelper(PID_t* pid, uint8_t btns, uint16_t sensorVal, uint16_t incr)
{
    uint32_t message = 0x00000000; // 32-bit buffer for setpoint and sensor value
    if (btns & 0x08)               // Up button pressed
        pid->setpoint += incr;     // Increase setpoint
    else if (btns & 0x04)          // Down button pressed
        pid->setpoint -= incr;     // Decrease setpoint
    message |= ((sensorVal << 0) | ((pid->setpoint) << 16)); // Combine values
    xQueueSend(fromPID, &message, mainDONT_BLOCK);           // Send to Display_Task
}

/**
 * @brief Cleans up system resources on failure or shutdown.
 * @details Suspends tasks, deletes tasks, queues, and semaphores, stops I2C,
 *          and disables GPIO interrupts to ensure a clean exit state.
 */
void cleanup_system(void)
{
    vTaskSuspendAll(); // Suspend all tasks to halt execution
    // Delete tasks if they exist
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
    // Delete queues if they exist
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
    // Delete semaphores if they exist
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
    // Stop I2C peripheral to free hardware resources
    XIic_Stop(&IicInstance);
    // Disable GPIO interrupts to prevent stray triggers
    XGpio_InterruptDisable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalDisable(&xInputGPIOInstance);
    xil_printf("System cleanup completed\r\n"); // Log completion
}

/**
 * @brief FreeRTOS task to handle all I2C transactions in a centralized manner.
 * @details Acts as a dedicated I2C manager, processing requests from i2c_request_queue
 *          for reading TSL2561 and BME280 sensors or writing to the OLED display.
 *          Ensures thread-safe I2C bus access by using a semaphore and handles
 *          replies via requester-specified queues. Runs indefinitely with a
 *          blocking receive to wait for requests.
 * @param p Unused parameter required by FreeRTOS task signature (ignored).
 */
void I2C_Task(void* p __attribute__((unused)))
{
    i2c_request_t req; // Local structure to hold incoming I2C request details

    while (1) // Infinite loop to continuously process I2C requests
    {
        // Wait for a request from i2c_request_queue with no timeout (blocks indefinitely)
        if (xQueueReceive(i2c_request_queue, &req, portMAX_DELAY) == pdPASS)
        {
            // Acquire exclusive access to the I2C bus, waiting as long as necessary
            xSemaphoreTake(i2c_sem, portMAX_DELAY);

            // Process the request based on its type (defined in main.h as i2c_request_type_t)
            switch (req.type)
            {
                // Handle TSL2561 Channel 0 read (visible + IR light)
                case READ_TSL2561_CH0:
                    // Read raw value from TSL2561 channel 0 using global I2C instance
                    *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
                    // Send the result (float) to the requester’s reply queue without blocking
                    xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
                    break;

                // Handle TSL2561 Channel 1 read (IR only)
                case READ_TSL2561_CH1:
                    // Read raw value from TSL2561 channel 1 using global I2C instance
                    *req.result = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
                    // Send the result (float) to the requester’s reply queue without blocking
                    xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
                    break;

                // Handle BME280 sensor reads (temperature, humidity, or pressure)
                case READ_BME280_TEMP:
                case READ_BME280_HUM:
                case READ_BME280_PRESS:
                {
                    // Local structure to store uncompensated BME280 data (raw 20-bit or 16-bit
                    // values)
                    struct bme280_uncomp_data uncomp_data;
                    // Buffer for raw data from BME280 (8 bytes: 3 for pressure, 3 for temp, 2 for
                    // humidity)
                    uint8_t buffer[8];
                    // Status variable to track I2C send/receive operations
                    int status;

                    // Set buffer[0] to REG_DATA, the starting register for BME280 measurement data
                    buffer[0] = REG_DATA;
                    // Send the register address to BME280 with a repeated start condition
                    status = XIic_Send(
                        IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START);
                    // Read 8 bytes of raw data from BME280, completing the transaction with a stop
                    status +=
                        XIic_Recv(IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP);

                    // Check if I2C transaction succeeded (1 byte sent + 8 bytes received = 9)
                    if (status == 9)
                    {
                        // Parse 20-bit pressure from first 3 bytes (MSB shifted left, LSB bits
                        // aligned)
                        uncomp_data.pressure = ((uint32_t) buffer[0] << 12) |
                                               ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
                        // Parse 20-bit temperature from next 3 bytes (same bit-shifting pattern)
                        uncomp_data.temperature = ((uint32_t) buffer[3] << 12) |
                                                  ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
                        // Parse 16-bit humidity from last 2 bytes (simple concatenation)
                        uncomp_data.humidity = ((uint32_t) buffer[6] << 8) | buffer[7];

                        // Compensate raw data based on request type, using global calib_data
                        if (req.type == READ_BME280_TEMP)
                            *req.result = (float) compensate_temperature(&uncomp_data, &calib_data);
                        else if (req.type == READ_BME280_HUM)
                            *req.result = (float) compensate_humidity(&uncomp_data, &calib_data);
                        else if (req.type == READ_BME280_PRESS)
                            *req.result = (float) compensate_pressure(&uncomp_data, &calib_data);
                        // Send compensated result (float) to requester’s queue without blocking
                        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
                    }
                    else
                    {
                        // On failure, return 0.0f as a fallback value
                        *req.result = 0.0f;
                        xQueueSend(req.reply_queue, req.result, mainDONT_BLOCK);
                        // Log error with status for debugging (e.g., I2C bus issue or sensor
                        // failure)
                        xil_printf("[ERROR] BME280 read failed in I2C_Task, status: %d\r\n",
                                   status);
                    }
                    break;
                }

                // Handle OLED display command writes
                case WRITE_LCD_CMD:
                {
                    // Create a 2-byte buffer: control byte (OLED_CMD) followed by the command
                    uint8_t buffer[2] = {OLED_CMD, req.cmd};
                    // Send the command directly to OLED’s I2C address with a stop condition
                    XIic_Send(IicInstance.BaseAddress, OLED_I2C_ADDR, buffer, 2, XIIC_STOP);
                    // If requester provided a reply queue, send a 1-byte acknowledgment
                    if (req.reply_queue)
                    {
                        uint8_t reply = 1;
                        xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
                    }
                    break;
                }

                // Handle OLED display data writes
                case WRITE_LCD_DATA:
                {
                    // Create a 129-byte buffer: 1 control byte + up to 128 data bytes
                    uint8_t buffer[129];
                    buffer[0] = OLED_DATA; // Set control byte indicating data
                    // Cap data length at 128 bytes (OLED page size) to avoid overflow
                    uint32_t bytes_to_send = (req.len > 128) ? 128 : req.len;
                    // Copy requester’s data into buffer starting at index 1
                    memcpy(&buffer[1], req.data, bytes_to_send);
                    // Send buffer (control byte + data) to OLED with a stop condition
                    XIic_Send(IicInstance.BaseAddress,
                              OLED_I2C_ADDR,
                              buffer,
                              bytes_to_send + 1,
                              XIIC_STOP);
                    // If requester provided a reply queue, send a 1-byte acknowledgment
                    if (req.reply_queue)
                    {
                        uint8_t reply = 1;
                        xQueueSend(req.reply_queue, &reply, mainDONT_BLOCK);
                    }
                    break;
                }

                // Handle unrecognized request types
                default:
                    // Log error with request type for debugging unexpected cases
                    xil_printf("[ERROR] Unknown I2C request type: %d\r\n", req.type);
                    break;
            }

            // Release I2C bus semaphore after processing to allow other tasks access
            xSemaphoreGive(i2c_sem);
        }
    }
}