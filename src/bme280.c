/*
 * bme280.c - BME280 sensor driver implementation for FreeRTOS
 *
 * Purpose: Provides functions to initialize, configure, and read data from the
 *          BME280 sensor, including compensation of raw data.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 */

#include "bme280.h"

/*
 * bme_init - Initialize the BME280 sensor over I2C
 * @iic: Pointer to the I2C controller instance
 * Returns: 0 on success, -1 on failure
 *
 * Sets up I2C communication with the BME280 sensor and verifies its identity
 * by reading the chip ID register. This ensures the sensor is present and
 * communicating correctly before further operations.
 */
int bme_init ( XIic* iic )
{
    uint8_t buffer[ 1 ];  // Single-byte buffer for chip ID read
    int     status;       // Return status for I2C operations

    // Configure the I2C controller with the BME280's address
    status = XIic_SetAddress ( iic, XII_ADDR_TO_SEND_TYPE, BME280_I2C_ADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Failed to set I2C address\r\n" );
        return -1;  // Indicate failure if address cannot be set
    }

    // Set the register address to read the chip ID
    buffer[ 0 ] = REG_CHIP_ID;
    // Send the register address with repeated start to prepare for reading
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    // Receive the chip ID value from the sensor
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_STOP );

    // Verify the received chip ID matches the expected BME280 ID
    if ( buffer[ 0 ] != BME280_CHIP_ID )
    {
        xil_printf ( "[ERROR] Wrong Chip ID: 0x%02X\r\n", buffer[ 0 ] );
        return -1;  // Return error if the chip ID doesn't match
    }

    return 0;  // Success: Sensor is initialized and verified
}

/*
 * bme_read_calibration_data - Retrieve calibration data from BME280
 * @iic: Pointer to the I2C controller instance
 * @calib: Pointer to structure where calibration data will be stored
 * Returns: 0 on success
 *
 * Reads factory-stored calibration coefficients from the BME280 sensor for
 * temperature, pressure, and humidity compensation. These values are critical
 * for converting raw sensor readings into accurate measurements.
 */
int bme_read_calibration_data ( XIic* iic, struct bme280_calib_data* calib )
{
    uint8_t buffer[ 26 ];  // Buffer to hold calibration data (26 bytes for temp/pressure, 7 for
                           // humidity)

    // Read temperature and pressure calibration data (26 bytes)
    buffer[ 0 ] = REG_TEMP_PRESS_CALIB;  // Starting register for temp/pressure calibration
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 26, XIIC_STOP );

    // Extract temperature calibration coefficients (unsigned and signed 16-bit values)
    calib->dig_t1 =
        (uint16_t) ( buffer[ 1 ] << 8 ) | buffer[ 0 ];  // T1: Unsigned temperature coefficient
    calib->dig_t2 =
        (int16_t) ( buffer[ 3 ] << 8 ) | buffer[ 2 ];  // T2: Signed temperature coefficient
    calib->dig_t3 =
        (int16_t) ( buffer[ 5 ] << 8 ) | buffer[ 4 ];  // T3: Signed temperature coefficient

    // Extract pressure calibration coefficients (unsigned and signed 16-bit values)
    calib->dig_p1 =
        (uint16_t) ( buffer[ 7 ] << 8 ) | buffer[ 6 ];  // P1: Unsigned pressure coefficient
    calib->dig_p2 =
        (int16_t) ( buffer[ 9 ] << 8 ) | buffer[ 8 ];  // P2: Signed pressure coefficient
    calib->dig_p3 =
        (int16_t) ( buffer[ 11 ] << 8 ) | buffer[ 10 ];  // P3: Signed pressure coefficient
    calib->dig_p4 =
        (int16_t) ( buffer[ 13 ] << 8 ) | buffer[ 12 ];  // P4: Signed pressure coefficient
    calib->dig_p5 =
        (int16_t) ( buffer[ 15 ] << 8 ) | buffer[ 14 ];  // P5: Signed pressure coefficient
    calib->dig_p6 =
        (int16_t) ( buffer[ 17 ] << 8 ) | buffer[ 16 ];  // P6: Signed pressure coefficient
    calib->dig_p7 =
        (int16_t) ( buffer[ 19 ] << 8 ) | buffer[ 18 ];  // P7: Signed pressure coefficient
    calib->dig_p8 =
        (int16_t) ( buffer[ 21 ] << 8 ) | buffer[ 20 ];  // P8: Signed pressure coefficient
    calib->dig_p9 =
        (int16_t) ( buffer[ 23 ] << 8 ) | buffer[ 22 ];  // P9: Signed pressure coefficient
    calib->dig_h1 = buffer[ 25 ];                        // H1: Unsigned 8-bit humidity coefficient

    // Read humidity calibration data (7 bytes)
    buffer[ 0 ] = REG_HUMIDITY_CALIB;  // Starting register for humidity calibration
    XIic_Send ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
    XIic_Recv ( iic->BaseAddress, BME280_I2C_ADDR, buffer, 7, XIIC_STOP );

    // Extract humidity calibration coefficients (mixed signed and unsigned values)
    calib->dig_h2 =
        (int16_t) ( buffer[ 1 ] << 8 ) | buffer[ 0 ];  // H2: Signed humidity coefficient
    calib->dig_h3 = buffer[ 2 ];                       // H3: Unsigned 8-bit humidity coefficient
    calib->dig_h4 = (int16_t) ( (int8_t) buffer[ 3 ] * 16 ) |
                    ( buffer[ 4 ] & 0x0F );  // H4: 12-bit signed (4 MSBs + 4 LSBs)
    calib->dig_h5 = (int16_t) ( (int8_t) buffer[ 5 ] * 16 ) |
                    ( buffer[ 4 ] >> 4 );  // H5: 12-bit signed (4 MSBs + 4 LSBs)
    calib->dig_h6 = (int8_t) buffer[ 6 ];  // H6: Signed 8-bit humidity coefficient

    return 0;  // Success: Calibration data successfully read and parsed
}

/*
 * BME280_Task - FreeRTOS task to configure and continuously read BME280 sensor
 * @pvParameters: Task parameters (unused)
 *
 * Configures the BME280 sensor for operation and enters an infinite loop to
 * read raw sensor data, compensate it using calibration values, and print the
 * results. Uses a semaphore for safe access in a multi-tasking environment.
 */
void BME280_Task ( void* pvParameters )
{
    struct bme280_uncomp_data uncomp_data;  // Structure to hold raw sensor readings
    uint8_t buffer[ 26 ];  // Buffer for I2C communication (max size for config/data)
    int     status;        // Status for I2C transaction results

    // Configure humidity control register (oversampling x1)
    buffer[ 0 ] = REG_CTRL_HUM;  // Humidity control register address
    buffer[ 1 ] = 0x01;          // Set humidity oversampling to 1x
    XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );

    // Configure measurement control register (temp x16, pressure x2, normal mode)
    buffer[ 0 ] = REG_CTRL_MEAS;  // Measurement control register address
    buffer[ 1 ] =
        ( 0x5 << 5 ) | ( 0x1 << 2 ) | 0x3;  // Temp x16 (0x5), Pressure x2 (0x1), Normal mode (0x3)
    XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );

    // Configure config register (standby 125ms, filter x16)
    buffer[ 0 ] = REG_CONFIG;                   // Config register address
    buffer[ 1 ] = ( 0x4 << 2 ) | ( 0x5 << 5 );  // Standby 125ms (0x4), Filter x16 (0x5)
    XIic_Send ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 2, XIIC_STOP );

    // Wait 500ms for sensor to stabilize after configuration
    vTaskDelay ( pdMS_TO_TICKS ( 500 ) );

    // Main task loop: Read and process sensor data indefinitely
    while ( 1 )
    {
        // Attempt to acquire semaphore with 100ms timeout for safe sensor access
        if ( xSemaphoreTake ( bme280_sem, pdMS_TO_TICKS ( 100 ) ) == pdTRUE )
        {
            // Read 8 bytes of sensor data (pressure, temp, humidity)
            buffer[ 0 ] = REG_DATA;  // Starting register for data output
            status      = XIic_Send (
                IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 1, XIIC_REPEATED_START );
            status += XIic_Recv ( IicInstance.BaseAddress, BME280_I2C_ADDR, buffer, 8, XIIC_STOP );

            // Check if I2C transactions completed successfully (1 byte sent + 8 bytes received = 9)
            if ( status != 9 )
            {
                xil_printf ( "[ERROR] Data read failed, status: %d\r\n", status );
            }
            else
            {
                // Parse 20-bit pressure value from 3 bytes (MSB, LSB, XLSB)
                uncomp_data.pressure = ( (uint32_t) buffer[ 0 ] << 12 ) |
                                       ( (uint32_t) buffer[ 1 ] << 4 ) | ( buffer[ 2 ] >> 4 );
                // Parse 20-bit temperature value from 3 bytes (MSB, LSB, XLSB)
                uncomp_data.temperature = ( (uint32_t) buffer[ 3 ] << 12 ) |
                                          ( (uint32_t) buffer[ 4 ] << 4 ) | ( buffer[ 5 ] >> 4 );
                // Parse 16-bit humidity value from 2 bytes (MSB, LSB)
                uncomp_data.humidity = ( (uint32_t) buffer[ 6 ] << 8 ) | buffer[ 7 ];

                // Print raw sensor values for debugging
                xil_printf ( "Raw T: %u, P: %u, H: %u\r\n",
                             uncomp_data.temperature,
                             uncomp_data.pressure,
                             uncomp_data.humidity );

                // Compensate raw values using calibration data
                int32_t temp =
                    compensate_temperature ( &uncomp_data, &calib_data );  // Temperature in 0.01°C
                uint32_t press =
                    compensate_pressure ( &uncomp_data, &calib_data );  // Pressure in Pa
                uint32_t hum =
                    compensate_humidity ( &uncomp_data, &calib_data );  // Humidity in 1024ths of %

                // Print compensated values with human-readable formatting
                xil_printf ( "Temp: %d.%02d C, Press: %u.%02u hPa, Hum: %u.%02u%%\r\n",
                             temp / 100,
                             abs ( temp % 100 ),  // Temp: degrees and hundredths
                             press / 100,
                             press % 100,  // Pressure: hPa and hundredths
                             hum / 1024,
                             ( hum % 1024 ) * 100 / 1024 );  // Humidity: % and hundredths
            }

            // Release semaphore to allow other tasks to access the sensor
            xSemaphoreGive ( bme280_sem );
        }
        else
        {
            xil_printf (
                "[ERROR] Semaphore timeout\r\n" );  // Log timeout if semaphore not acquired
        }

        // Delay 1 second before next reading cycle
        vTaskDelay ( pdMS_TO_TICKS ( 1000 ) );
    }
}

/*
 * compensate_temperature - Compensate raw temperature data
 * @uncomp_data: Pointer to structure with raw sensor readings
 * @calib_data: Pointer to structure with calibration coefficients
 * Returns: Compensated temperature in hundredths of degrees Celsius (e.g., 2530 = 25.30°C)
 *
 * Applies the BME280 temperature compensation formula from the datasheet,
 * using calibration data to convert raw temperature readings into accurate values.
 * Limits output to -40°C to 85°C range.
 */
int32_t compensate_temperature ( const struct bme280_uncomp_data* uncomp_data,
                                 struct bme280_calib_data*        calib_data )
{
    int32_t var1, var2, temperature;  // Intermediate variables for compensation

    // Step 1: Calculate var1 (linear term)
    var1 = (int32_t) ( ( uncomp_data->temperature / 8 ) - ( (int32_t) calib_data->dig_t1 * 2 ) );
    var1 = ( var1 * ( (int32_t) calib_data->dig_t2 ) ) / 2048;

    // Step 2: Calculate var2 (quadratic term)
    var2 = (int32_t) ( ( uncomp_data->temperature / 16 ) - ( (int32_t) calib_data->dig_t1 ) );
    var2 = ( ( ( var2 * var2 ) / 4096 ) * ( (int32_t) calib_data->dig_t3 ) ) / 16384;

    // Store fine temperature value (t_fine) for use in pressure/humidity compensation
    calib_data->t_fine = var1 + var2;

    // Final temperature calculation with scaling to hundredths of °C
    temperature = ( calib_data->t_fine * 5 + 128 ) / 256;

    // Clamp temperature to valid range: -40°C (-4000) to 85°C (8500)
    if ( temperature < -4000 )
        temperature = -4000;
    else if ( temperature > 8500 )
        temperature = 8500;

    return temperature;
}

/*
 * compensate_pressure - Compensate raw pressure data
 * @uncomp_data: Pointer to structure with raw sensor readings
 * @calib_data: Pointer to structure with calibration coefficients
 * Returns: Compensated pressure in Pascals (e.g., 101325 Pa = 1013.25 hPa)
 *
 * Applies the BME280 pressure compensation formula from the datasheet,
 * using calibration data and t_fine from temperature compensation to convert
 * raw pressure readings into accurate values. Limits output to 300-1100 hPa.
 */
uint32_t compensate_pressure ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib_data )
{
    int32_t  var1, var2, var3, var4;  // Intermediate variables for compensation
    uint32_t var5, pressure;          // Additional variables and final pressure

    // Step 1: Initial adjustment using t_fine
    var1 = ( ( (int32_t) calib_data->t_fine ) / 2 ) - (int32_t) 64000;

    // Step 2: Quadratic term with p6 coefficient
    var2 = ( ( ( var1 / 4 ) * ( var1 / 4 ) ) / 2048 ) * ( (int32_t) calib_data->dig_p6 );
    var2 = var2 + ( ( var1 * ( (int32_t) calib_data->dig_p5 ) ) * 2 );
    var2 = ( var2 / 4 ) + ( ( (int32_t) calib_data->dig_p4 ) * 65536 );

    // Step 3: Additional corrections
    var3 = ( calib_data->dig_p3 * ( ( ( var1 / 4 ) * ( var1 / 4 ) ) / 8192 ) ) / 8;
    var4 = ( ( (int32_t) calib_data->dig_p2 ) * var1 ) / 2;
    var1 = ( var3 + var4 ) / 262144;
    var1 = ( ( ( 32768 + var1 ) ) * ( (int32_t) calib_data->dig_p1 ) ) / 32768;

    // Step 4: Calculate pressure if var1 is non-zero (avoid division by zero)
    if ( var1 )
    {
        var5     = (uint32_t) ( (uint32_t) 1048576 ) - uncomp_data->pressure;
        pressure = ( (uint32_t) ( var5 - (uint32_t) ( var2 / 4096 ) ) ) * 3125;
        // Handle 32-bit overflow in pressure calculation
        if ( pressure < 0x80000000 )
            pressure = ( pressure << 1 ) / ( (uint32_t) var1 );
        else
            pressure = ( pressure / (uint32_t) var1 ) * 2;

        // Final adjustments with higher-order coefficients
        var1 = ( ( (int32_t) calib_data->dig_p9 ) *
                 ( (int32_t) ( ( ( pressure / 8 ) * ( pressure / 8 ) ) / 8192 ) ) ) /
               4096;
        var2 = ( ( (int32_t) ( pressure / 4 ) ) * ( (int32_t) calib_data->dig_p8 ) ) / 8192;
        pressure =
            (uint32_t) ( (int32_t) pressure + ( ( var1 + var2 + calib_data->dig_p7 ) / 16 ) );

        // Clamp pressure to valid range: 300 hPa (30000 Pa) to 1100 hPa (110000 Pa)
        if ( pressure < 30000 )
            pressure = 30000;
        else if ( pressure > 110000 )
            pressure = 110000;
    }
    else
    {
        pressure = 30000;  // Default to minimum value if var1 is zero
    }

    return pressure;
}

/*
 * compensate_humidity - Compensate raw humidity data
 * @uncomp_data: Pointer to structure with raw sensor readings
 * @calib_data: Pointer to structure with calibration coefficients
 * Returns: Compensated humidity in 1024ths of a percent (e.g., 51200 = 50%)
 *
 * Applies the BME280 humidity compensation formula from the datasheet,
 * using calibration data and t_fine from temperature compensation to convert
 * raw humidity readings into accurate values. Limits output to 0-100%.
 */
uint32_t compensate_humidity ( const struct bme280_uncomp_data* uncomp_data,
                               const struct bme280_calib_data*  calib_data )
{
    int32_t  var1, var2, var3, var4, var5;  // Intermediate variables for compensation
    uint32_t humidity;                      // Final compensated humidity value

    // Step 1: Adjust t_fine for humidity calculation
    var1 = calib_data->t_fine - ( (int32_t) 76800 );

    // Step 2: Linear term with raw humidity and h4/h5 coefficients
    var2 = (int32_t) ( uncomp_data->humidity * 16384 );
    var3 = (int32_t) ( ( (int32_t) calib_data->dig_h4 ) * 1048576 );
    var4 = ( (int32_t) calib_data->dig_h5 ) * var1;
    var5 = ( ( ( var2 - var3 ) - var4 ) + (int32_t) 16384 ) / 32768;

    // Step 3: Temperature-dependent terms with h6 and h3 coefficients
    var2 = ( var1 * ( (int32_t) calib_data->dig_h6 ) ) / 1024;
    var3 = ( var1 * ( (int32_t) calib_data->dig_h3 ) ) / 2048;
    var4 = ( ( var2 * ( var3 + (int32_t) 32768 ) ) / 1024 ) + (int32_t) 2097152;

    // Step 4: Final scaling with h2 coefficient
    var2 = ( ( var4 * ( (int32_t) calib_data->dig_h2 ) ) + 8192 ) / 16384;
    var3 = var5 * var2;

    // Step 5: Non-linear correction with h1 coefficient
    var4 = ( ( var3 / 32768 ) * ( var3 / 32768 ) ) / 128;
    var5 = var3 - ( ( var4 * ( (int32_t) calib_data->dig_h1 ) ) / 16 );

    // Clamp intermediate value to valid range
    var5 = ( var5 < 0 ? 0 : var5 );                  // Minimum 0
    var5 = ( var5 > 419430400 ? 419430400 : var5 );  // Maximum before scaling

    // Convert to 1024ths of a percent
    humidity = (uint32_t) ( var5 / 4096 );

    // Limit humidity to 100% (102400 in 1024ths)
    if ( humidity > 102400 ) humidity = 102400;

    return humidity;
}