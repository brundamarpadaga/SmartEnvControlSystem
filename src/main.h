#ifndef MAIN_H
#define MAIN_H

/* ===========================================================================
 * Standard C Library Includes
 * ===========================================================================
 */
#include <stdint.h>
#include <stdlib.h>

/* ===========================================================================
 * Xilinx BSP and Platform Includes
 * ===========================================================================
 */
#include "sleep.h"
#include "xgpio.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xtmrctr.h"

/* ===========================================================================
 * FreeRTOS Kernel Includes
 * ===========================================================================
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* ===========================================================================
 * Project-Specific Includes
 * ===========================================================================
 */
#include "bme280.h"
#include "lcd.h"
#include "nexys4IO.h"
#include "pidtask.h"
#include "platform.h"
#include "tsl2561.h"

/* ===========================================================================
 * Peripheral Definitions
 * ===========================================================================
 */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

#define BTN_CHANNEL 1 // GPIO channel for buttons
#define SW_CHANNEL 2  // GPIO channel for switches

/* ===========================================================================
 * Configuration Constants
 * ===========================================================================
 */
#define mainQUEUE_LENGTH 1             // Queue length for task communication
#define mainDONT_BLOCK ((TickType_t)0) // Non-blocking operation
#define QUEUE_TIMEOUT_TICKS pdMS_TO_TICKS(10)  // 10ms timeout for queue ops
#define DEBOUNCE_DELAY_TICKS pdMS_TO_TICKS(20) // 20ms debounce period

#define max_duty 255    // Maximum PWM duty cycle
#define min_duty 0      // Minimum PWM duty cycle
#define lux_mask 0xFFFF // Mask for 16-bit lux values

/* ===========================================================================
 * Logging Levels
 * ===========================================================================
 */
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARNING 2
#define LOG_LEVEL_INFO 3
#define DEBUG_LEVEL LOG_LEVEL_INFO

/* ===========================================================================
 * Bit Masks
 * ===========================================================================
 */
#define BUTTON_MASK 0x1F // 5 button bits
#define SWITCH_MASK 0xFF // 8 switch bits

/* ===========================================================================
 * Utility Macro
 * ===========================================================================
 */
#define UPDATE_SATURATING(val, inc, min_val, max_val, increase)                \
  do {                                                                         \
    if (increase) {                                                            \
      if ((val + inc) < max_val)                                               \
        val += inc;                                                            \
      else                                                                     \
        val = max_val;                                                         \
    } else {                                                                   \
      if (val > inc)                                                           \
        val -= inc;                                                            \
      else                                                                     \
        val = min_val;                                                         \
    }                                                                          \
  } while (0)

/* ===========================================================================
 * I2C Request Types and Structure
 * ===========================================================================
 */
typedef enum {
  READ_TSL2561_CH0,
  READ_TSL2561_CH1,
  READ_BME280_TEMP,
  READ_BME280_HUM,
  READ_BME280_PRESS,
  WRITE_LCD_CMD,
  WRITE_LCD_DATA
} i2c_request_type_t;

typedef struct {
  i2c_request_type_t type;   // Type of I2C operation
  float *result;             // Result pointer for reads (NULL for writes)
  uint8_t *data;             // Data for LCD writes (NULL otherwise)
  uint32_t len;              // Length of data for LCD writes
  uint8_t cmd;               // Command for LCD writes
  QueueHandle_t reply_queue; // Queue for reply (NULL for writes)
} i2c_request_t;

/* ===========================================================================
 * Global Instances
 * ===========================================================================
 */
extern SemaphoreHandle_t binary_sem;
extern SemaphoreHandle_t bme280_sem;
extern SemaphoreHandle_t oled_sem;
extern SemaphoreHandle_t i2c_sem;
extern XGpio xInputGPIOInstance;
extern QueueHandle_t toPID;
extern QueueHandle_t fromPID;
extern QueueHandle_t i2c_request_queue;
extern XIic IicInstance;
extern struct bme280_calib_data calib_data;

/* ===========================================================================
 * Function Declarations
 * ===========================================================================
 */
void prvSetupHardware(void);
void gpio_intr(void *pvUnused);
int do_init(void);
void Parse_Input_Task(void *p);
void PID_Task(void *p);
void Display_Task(void *p);
uint8_t correctedSignal(uint8_t enviro, float pidOut, bool fanCtrl);
void displayHelper(PID_t *pid, uint8_t btns, uint16_t sensorVal, uint16_t incr);
void cleanup_system(void);
void I2C_Task(void *p);

#endif /* MAIN_H */