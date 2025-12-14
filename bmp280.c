#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmp280.h"

static char *TAG = "BMP280";

#define I2C_MASTER_TIMEOUT_MS       1000

// BMP280 Register Addresses
/* Raw temperature data */
#define BMP280_REG_ATTR_TEMP_XLSB   0xFC /* bits: 7-4 = ut[3:0] */
#define BMP280_REG_ATTR_TEMP_LSB    0xFB /* bits: 7-0 = ut[11:4] */
#define BMP280_REG_ATTR_TEMP_MSB    0xFA /* bits: 7-0 = ut[19:12] */
/* Raw pressure data */
#define BMP280_REG_ATTR_PRESS_XLSB  0xF9 /* bits: 7-4 = up[3:0] */
#define BMP280_REG_ATTR_PRESS_LSB   0xF8 /* bits: 7-0 = up[11:4] */
#define BMP280_REG_ATTR_PRESS_MSB   0xF7 /* bits: 7-0 = up[19:12] */
/* Other config registries */
#define BMP280_REG_ATTR_CONFIG      0xF5 /* bits: 7-5 = t_sb[2:0] standby time in normal mode; 4-2 = filter[2:0] time constant of IIR filter; 0 = spi3w_en[0] enable 3-wire SPI */
#define BMP280_REG_ATTR_CTRL_MEAS   0xF4 /* bits: 7-5 = osrs_t[2:0] temperature oversampling; 4-2 = osrs_p[2:0] pressure oversampling; 1-0 = mode[1:0] device power mode */
#define BMP280_REG_ATTR_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_ATTR_RESET       0xE0 /* bits: 7-0 = reset[7:0] writing 0xB6 resets the device */
#define BMP280_REG_ATTR_ID          0xD0 /* bits: 7-0 = chip_id[7:0] = 0x58 */
#define BMP280_REG_ATTR_CALIB_START 0x88 /* Start address of the calibration data */
#define BMP280_REG_ATTR_CALIB_END   0xA1 /* End address of the calibration data */

#define BMP280_RESET_VALUE          0xB6

/*
  Initializes given i2c bus and master device
  Arguments:
  - <i2c_master_bus_handle_t*> bus_handle: Pointer to the i2c master bus handle to be initialized
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the i2c master device handle to be initialized
  - <bmp280_params_t*> params: Pointer to the BMP280 initialization parameters structure containing:
    - <i2c_port_num_t> params->i2c_port: I2C port number to be used
    - <gpio_num_t> params->sda_io_num: GPIO number to be used for SDA signal
    - <gpio_num_t> params->scl_io_num: GPIO number to be used for SCL signal
    - <bool> params->use_lp_i2c: Whether to use LP I2C peripheral (if supported by the SoC)
    - <i2c_clock_source_t> params->clk_source: Clock source to be used by the I2C master bus
    - <bool> params->use_primary_address: Whether to use primary (0x76) or secondary (0x77) I2C address of the BMP280
    - <uint32_t> params->i2c_master_freq_hz: I2C master clock frequency in Hz
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, bmp280_params_t *params) {
    esp_err_t result_code;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = params->i2c_port,
        .sda_io_num = params->sda_io_num,
        .scl_io_num = params->scl_io_num,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (params->use_lp_i2c) {
        #if SOC_LP_I2C_SUPPORTED
            bus_config.lp_source_clk = params->clk_source;
        #else
            ESP_LOGE(TAG, "LP I2C not supported on this SoC");
            return ESP_ERR_INVALID_ARG;
        #endif
    } else {
        bus_config.clk_source = params->clk_source;
    }

    result_code = i2c_new_master_bus(&bus_config, bus_handle);
    if (result_code != ESP_OK) return result_code;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = params->use_primary_address ? 0x76 : 0x77,
        .scl_speed_hz = params->i2c_master_freq_hz,
    };
    result_code = i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
    if (result_code != ESP_OK) {
      i2c_del_master_bus(*bus_handle); // Try to deinitialize created i2c master bus
      return result_code;
    }

    return ESP_OK;
}

/*
  Deinitializes given i2c bus and master device. It is recommended to put the device into sleep mode prior to calling this function.
  Arguments:
  - <i2c_master_bus_handle_t*> bus_handle: Pointer to the i2c master bus handle to be initialized
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the i2c master device handle to be initialized
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_deinit(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    esp_err_t result_code;

    result_code = i2c_master_bus_rm_device(*dev_handle);
    if (result_code != ESP_OK) return result_code;

    result_code = i2c_del_master_bus(*bus_handle);
    if (result_code != ESP_OK) return result_code;

    return ESP_OK;
}

/*
  Utility function to read 8-bit value using the registry address
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t> reg_addr: Register address to read from
  - <uint8_t*> data: Pointer where the read data will be stored
  - <size_t> len: Number of bytes to read
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
static esp_err_t register_read(i2c_master_dev_handle_t * dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(*dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/*
  Utility function to read 16-bit value using the registry address and write the value to the address given
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t> reg_addr: Register address to read from
  - <uint16_t*> data: Pointer where the read data will be stored
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
static esp_err_t register_read_16(i2c_master_dev_handle_t * dev_handle, uint8_t reg_addr, uint16_t *data) {
    uint8_t read_data[] = { 0, 0 };

    esp_err_t ret = register_read(dev_handle, reg_addr, read_data, sizeof(read_data));
    if (ret != ESP_OK) return ret;
    *data = read_data[0] | (read_data[1] << 8);

    return ESP_OK;
}

/*
  Utility function to write the value to the device using the registry address and value
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t> reg_addr: Register address that will be overwritten
  - <uint8_t> value: Value to be written to the register
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
inline static esp_err_t register_write(i2c_master_dev_handle_t * dev_handle, uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2] = { reg_addr, value };
    return i2c_master_transmit(*dev_handle, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}

/*
  Retrieves the BMP280 chip id
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t*> chip_id: Address where the read chip id will be stored
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_get_chip_id(i2c_master_dev_handle_t * dev_handle, uint8_t * chip_id) {
    return register_read(dev_handle, BMP280_REG_ATTR_ID, chip_id, 1);
}

/*
  Resets the chip by writing the reset value to the reset register
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_reset_chip(i2c_master_dev_handle_t * dev_handle) {
    return register_write(dev_handle, BMP280_REG_ATTR_RESET, BMP280_RESET_VALUE);
}

/*
  Forces a single measurement using forced mode. Config should be set prior to calling this function.
  Chip will return to sleep mode after the measurement is done.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_force_measurement(i2c_master_dev_handle_t * dev_handle) {
    uint8_t mode;
    esp_err_t result_code = bmp280_get_mode_raw(dev_handle, &mode);
    if (result_code != ESP_OK) return result_code;

    mode &= ~0b11; // Clear the bytes 1-0 (measurement mode bits)
    mode |= BMP280_MODE_FORCED; // Set the mode to forced
    result_code = bmp280_set_mode_raw(dev_handle, mode);
    if (result_code != ESP_OK) return result_code;

    return ESP_OK;
}

/*
  Checks if the NVM data are being copied to image registers which happens during power-on-reset and before every conversion.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <bool*> value: Pointer where the read value will be stored (true if updating, false otherwise)
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_is_updating_nvm(i2c_master_dev_handle_t * dev_handle, bool * value) {
    esp_err_t result_code;
    uint8_t status;

    result_code = register_read(dev_handle, BMP280_REG_ATTR_STATUS, &status, 1);
    if (result_code != ESP_OK) return result_code;

    *value = status & 0b1; // im_update bit

    return ESP_OK;
}

/*
  Checks if the device is currently measuring.
  The check is done by reading the measuring bit from the status register and checking if the device is in forced mode (it should return to the sleep mode after the measure is completed).
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <bool*> value: Pointer where the measurement state will be stored (true if measuring, false otherwise)
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_is_measuring(i2c_master_dev_handle_t * dev_handle, bool * value) {
    esp_err_t result_code;
    uint8_t status;
    BMP280Mode device_mode;

    result_code = register_read(dev_handle, BMP280_REG_ATTR_STATUS, &status, 1);
    if (result_code != ESP_OK) return result_code;

    result_code = bmp280_get_mode(dev_handle, NULL, NULL, &device_mode);
    if (result_code != ESP_OK) return result_code;

    *value = (device_mode == BMP280_MODE_FORCED) || (status & 0b1000); // Forced measure or measuring bit active

    return ESP_OK;
}

/*
  Waits synchronously for the measurement to be completed. It utilizes the `bmp280_is_measuring` function to check the measurement state and rechecks every 10 ms until the measurement is done.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_wait_measurement_done(i2c_master_dev_handle_t * dev_handle) {
    esp_err_t result_code;
    bool measuring = true;
    do {
        result_code = bmp280_is_measuring(dev_handle, &measuring);
        if (result_code != ESP_OK) return result_code;

        if (!measuring) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (true);

    return ESP_OK;
}

/*
  Sets raw config value (filter & standby values compressed into uint8).
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t> config: Config value to be written
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_set_config_raw(i2c_master_dev_handle_t * dev_handle, uint8_t config) {
    return register_write(dev_handle, BMP280_REG_ATTR_CONFIG, config);
}

/*
  Reads raw config (filter & standby values) from the registry.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t*> config: Address where the read config value will be stored
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_get_config_raw(i2c_master_dev_handle_t * dev_handle, uint8_t * config) {
    return register_read(dev_handle, BMP280_REG_ATTR_CONFIG, config, 1);
}

/*
  Sets the BMP280 config (filter & standby) using the enums provided.
  Internally it calls the `bmp280_set_config_raw` function to write the compressed config value.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <BMP280Filter> filter: Filter enum value to be set
  - <BMP280StandbyTime> standby: Standby time enum value to be set
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_set_config(i2c_master_dev_handle_t * dev_handle, BMP280Filter filter, BMP280StandbyTime standby) {
    uint8_t config = (standby << 5) | (filter << 2);
    return bmp280_set_config_raw(dev_handle, config);
}

/*
  Reads the BMP280 config (filter & standby) to the enums addresses provided.
  Internally it calls the `bmp280_get_config_raw` function to read the compressed config value.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <BMP280Filter*> filter: Address where the read filter enum value will be stored (Won't be read if NULL)
  - <BMP280StandbyTime*> standby: Address where the read standby time enum value will be stored (Won't be read if NULL)
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_get_filter_standby(i2c_master_dev_handle_t * dev_handle, BMP280Filter * filter, BMP280StandbyTime * standby) {
    uint8_t config;
    esp_err_t result_code = bmp280_get_config_raw(dev_handle, &config);
    if (result_code != ESP_OK) return result_code;

    if (standby != NULL) *standby = (config >> 5) & 0b111;
    if (filter != NULL) *filter = (config >> 2) & 0b111;

    return ESP_OK;
}

/*
  Sets raw mode (temperature sampling, pressure sampling and device mode values compressed into uint8).
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t> mode: Mode value to be written
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_set_mode_raw(i2c_master_dev_handle_t * dev_handle, uint8_t mode) {
    return register_write(dev_handle, BMP280_REG_ATTR_CTRL_MEAS, mode);
}

/*
  Reads raw mode (temperature sampling, pressure sampling and device mode values).
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <uint8_t*> mode: Address where the read mode value will be stored
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_get_mode_raw(i2c_master_dev_handle_t * dev_handle, uint8_t * mode) {
    return register_read(dev_handle, BMP280_REG_ATTR_CTRL_MEAS, mode, 1);
}

/*
  Sets the BMP280 mode (temperature sampling, pressure sampling and device mode) using the enums provided.
  Internally it calls the `bmp280_set_mode_raw` function to write the compressed mode value.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <BMP280OversamplingMode> temperature_oversampling: Temperature oversampling enum value to be set
  - <BMP280OversamplingMode> pressure_oversampling: Pressure oversampling enum value to be set
  - <BMP280Mode> device_mode: Device mode enum value to be set
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_set_mode(i2c_master_dev_handle_t * dev_handle, BMP280OversamplingMode temperature_oversampling, BMP280OversamplingMode pressure_oversampling, BMP280Mode device_mode) {
    uint8_t ctrl = (temperature_oversampling << 5) | (pressure_oversampling << 2) | (device_mode);
    return bmp280_set_mode_raw(dev_handle, ctrl);
}

/*
  Reads the BMP280 mode (temperature sampling, pressure sampling and device mode) to enums addresses provided.
  Internally it calls the `bmp280_get_mode_raw` function to read the compressed mode value from proper registry.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <BMP280OversamplingMode*> temperature_oversampling: Address where the read temperature oversampling enum value will be stored (Won't be read if NULL)
  - <BMP280OversamplingMode*> pressure_oversampling: Address where the read pressure oversampling enum value will be stored (Won't be read if NULL)
  - <BMP280Mode*> device_mode: Address where the read device mode enum value will be stored (Won't be read if NULL)
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_get_mode(i2c_master_dev_handle_t * dev_handle, BMP280OversamplingMode * temperature_oversampling, BMP280OversamplingMode * pressure_oversampling, BMP280Mode * device_mode) {
    uint8_t ctrl;
    esp_err_t result_code = bmp280_get_mode_raw(dev_handle, &ctrl);
    if (result_code != ESP_OK) return result_code;

    if (temperature_oversampling != NULL) *temperature_oversampling = (ctrl >> 5) & 0b111;
    if (pressure_oversampling != NULL) *pressure_oversampling = (ctrl >> 2) & 0b111;
    if (device_mode != NULL) *device_mode = ctrl & 0b11;

    return ESP_OK;
}

/*
  Performs reading of raw pressure and temperature data from the BMP280 sensor.
  It performs the reading of 6 bytes in bulk to minimize I2C transactions. If the registry is updated during the read, the data will be consistent thanks to the BMP280 data shadowing feature.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <int32_t*> raw_pressure: Pointer where the read raw pressure value will be stored (Won't be read if NULL)
  - <int32_t*> raw_temperature: Pointer where the read raw temperature value will be stored (Won't be read if NULL)
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason  
*/
esp_err_t bmp280_read_data_raw(i2c_master_dev_handle_t * dev_handle, int32_t * raw_pressure, int32_t * raw_temperature) {
    uint8_t raw_data[6];
    // Reads BMP280_REG_ATTR_PRESS_MSB to BMP280_REG_ATTR_TEMP_XLSB
    esp_err_t result_code = register_read(dev_handle, BMP280_REG_ATTR_PRESS_MSB, raw_data, sizeof(raw_data));
    if (result_code != ESP_OK) return result_code;

    if (raw_pressure != NULL) *raw_pressure = ((int32_t) raw_data[0] << 12) | ((int32_t) raw_data[1] << 4) | (raw_data[2] >> 4);
    if (raw_temperature != NULL) *raw_temperature = ((int32_t) raw_data[3] << 12) | ((int32_t) raw_data[4] << 4) | (raw_data[5] >> 4);

    return ESP_OK;
}

/*
  Applies compensation formulas to transform retrieved raw temperature values into actual temperature value.
  The output temperature is in degree Celsius multiplied by 100 (e.g., a value of 2534 represents 25.34 °C).
  It is based on the documented compensation formula provided in the BMP280 datasheet.
  Arguments:
  - <int32_t> adc_temp: Raw temperature value retrieved from the sensor
  - <bmp280_calibration_data_t*> data: Pointer to the calibration data structure
  Returns:
  - <int32_t> Compensated temperature value in degree Celsius multiplied by 100  
*/
int32_t bmp280_compensate_temperature(int32_t adc_temp, bmp280_calibration_data_t * data) {
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t) data->dig_T1 << 1))) * (int32_t) data->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t) data->dig_T1) * ((adc_temp >> 4) - (int32_t) data->dig_T1)) >> 12) * (int32_t) data->dig_T3) >> 14;

    data->temp_fine = var1 + var2;
    return (data->temp_fine * 5 + 128) >> 8;
}

/*
  Applies the temperature fine value from the given temperature in degree Celsius multiplied by 100.
  It may be used to set the temp_fine value in the calibration data structure when the BMP280 temperature reading is disabled but temperature is known from another source.
  Arguments:
  - <int32_t> temperature: Temperature value in degree Celsius multiplied by 100
  - <bmp280_calibration_data_t*> data: Pointer to the calibration data structure
*/
void bmp280_apply_temperature_fine(int32_t temperature, bmp280_calibration_data_t * data) {
    data->temp_fine = (((temperature << 8) - 128) / 5); // Reverse of final temperature calculation
}

/*
  Applies compensation formulas to transform retrieved raw pressure values into actual pressure value.
  The output pressure is in Pa (Pascal) multiplied by 256 (e.g., a value of 100125 represents 100125 / 256 = 391.6 Pa).
  It is based on the documented compensation formula provided in the BMP280 datasheet.
  NOTE: Requires that `bmp280_compensate_temperature` was called first to retrieve the temperature fine value used for calibration or it was applied using the `bmp280_apply_temperature_fine` function.
  Arguments:
  - <int32_t> adc_pres: Raw pressure value retrieved from the sensor
  - <bmp280_calibration_data_t*> data: Pointer to the calibration data structure
  Returns:
  - <uint32_t> Compensated pressure value in Pa multiplied by 256
*/
uint32_t bmp280_compensate_pressure(int32_t adc_pres, bmp280_calibration_data_t * data) {
    int64_t var1, var2;

    var1 = (int64_t) data->temp_fine - 128000;
    var2 = var1 * var1 * (int64_t) data->dig_P6;
    var2 = var2 + ((var1 * (int64_t) data->dig_P5) << 17);
    var2 = var2 + (((int64_t) data->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) data->dig_P3) >> 8) + ((var1 * (int64_t) data->dig_P2) << 12);
    var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) data->dig_P1) >> 33;
    
    if (var1 == 0) return 0; // avoid exception caused by division by zero

    int64_t p = 1048576 - adc_pres;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t) data->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t) data->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t) data->dig_P7 << 4);
    return (uint32_t) p;
}

/*
  Reads the BMP280 calibration data from the sensor and fills the provided calibration data structure.
  It is recommended to call this function once during the initialization phase and reuse the calibration data for further compensation calculations.
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <bmp280_calibration_data_t*> data: Pointer to the calibration data structure to be filled
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason
*/
esp_err_t bmp280_read_calibration_data(i2c_master_dev_handle_t * dev_handle, bmp280_calibration_data_t * data) {
    esp_err_t result_code;
    #define READ_CALIB_DATA_FAIL_CHECK(offset, field) \
        result_code = register_read_16(dev_handle, BMP280_REG_ATTR_CALIB_START + offset, field); \
        if (result_code != ESP_OK) return result_code;

    READ_CALIB_DATA_FAIL_CHECK(0, &data->dig_T1);
    READ_CALIB_DATA_FAIL_CHECK(2, (uint16_t *) &data->dig_T2);
    READ_CALIB_DATA_FAIL_CHECK(4, (uint16_t *) &data->dig_T3);
    READ_CALIB_DATA_FAIL_CHECK(6, &data->dig_P1);
    READ_CALIB_DATA_FAIL_CHECK(8, (uint16_t *) &data->dig_P2);
    READ_CALIB_DATA_FAIL_CHECK(10, (uint16_t *) &data->dig_P3);
    READ_CALIB_DATA_FAIL_CHECK(12, (uint16_t *) &data->dig_P4);
    READ_CALIB_DATA_FAIL_CHECK(14, (uint16_t *) &data->dig_P5);
    READ_CALIB_DATA_FAIL_CHECK(16, (uint16_t *) &data->dig_P6);
    READ_CALIB_DATA_FAIL_CHECK(18, (uint16_t *) &data->dig_P7);
    READ_CALIB_DATA_FAIL_CHECK(20, (uint16_t *) &data->dig_P8);
    READ_CALIB_DATA_FAIL_CHECK(22, (uint16_t *) &data->dig_P9);

    #undef READ_CALIB_DATA_FAIL_CHECK

    return ESP_OK;
}

/*
  Performs synchronous reading of temperature and pressure data from the BMP280 sensor according to the saved device settings.
  It compensates the retrieved raw values using the calibration data read from the sensor and returns the values in human-readable format (Celsius degrees and pascals).
  Arguments:
  - <i2c_master_dev_handle_t*> dev_handle: Pointer to the BMP280 i2c master device handle
  - <bmp280_calibration_data_t*> data: Pointer to the calibration data structure to be filled
  Returns:
  - <esp_err_t> ESP_OK on success, otherwise an error code indicating the failure reason
*/
esp_err_t bmp280_read(i2c_master_dev_handle_t * dev_handle, bmp280_calibration_data_t * calib_data, bmp280_data_t * bmp280_data) {
    esp_err_t result_code;

    BMP280OversamplingMode temp_oversampling, pres_oversampling;
    result_code = bmp280_get_mode(dev_handle, &temp_oversampling, &pres_oversampling, NULL);
    if (result_code != ESP_OK) return result_code;
    if (temp_oversampling == BMP280_DISABLED || pres_oversampling == BMP280_DISABLED) {
        ESP_LOGE(TAG, "Temperature or pressure oversampling is disabled, cannot perform proper reading. Please consider enabling them or performing the read manually.");
        return ESP_ERR_INVALID_STATE;
    }

    // Wait for measurement to be done
    result_code = bmp280_wait_measurement_done(dev_handle);
    if (result_code != ESP_OK) return result_code;

    // Read raw temperature and pressure
    int32_t raw_pressure, raw_temperature;
    result_code = bmp280_read_data_raw(dev_handle, &raw_pressure, &raw_temperature);
    if (result_code != ESP_OK) return result_code;

    int32_t temperature = bmp280_compensate_temperature(raw_temperature, calib_data);
    uint32_t pressure = bmp280_compensate_pressure(raw_pressure, calib_data);

    bmp280_data->temperature = (float) temperature / 100;
    bmp280_data->pressure = (float) pressure / 256;

    return ESP_OK;
}