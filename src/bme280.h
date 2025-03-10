#ifndef BME280_H
#define BME280_H

/* Header includes */
#include "bme280_defs.h"
#include "xiic.h"
#include "stdlib.h"

/* BME280 I2C address */
#define BME280_I2C_ADDR 0x76

/* Register addresses */
#define REG_CHIP_ID 0xD0
#define REG_TEMP_PRESS_CALIB 0x88
#define REG_HUMIDITY_CALIB 0xE1
#define REG_CTRL_HUM 0xF2
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_DATA 0xF7

// Function Prototypes
int bme280_initialise(XIic * IicInstance);
void bme280_configure(XIic * IicInstance);
void bme_read8Bytes(XIic* IicInstance, struct bme280_uncomp_data * uncomp_data,struct bme280_calib_data* calib_data);
struct bme280_calib_data  readTempPressureHumidityCalib(XIic* IicInstance);
int32_t compensate_temperature(const struct bme280_uncomp_data* uncomp_data, struct bme280_calib_data* calib_data);
uint32_t compensate_pressure(const struct bme280_uncomp_data* uncomp_data, const struct bme280_calib_data* calib_data);
uint32_t compensate_humidity(const struct bme280_uncomp_data* uncomp_data, const struct bme280_calib_data* calib_data);



#endif
