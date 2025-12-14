#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

// Suggested BMP280 I2C Master clock frequency
#define BMP280_I2C_MASTER_FREQ_HZ          100000  // 100kHz

// Enums
/*
  Represents the BMP280 operating modes
*/
typedef enum {
  BMP280_MODE_SLEEP = 0b00, // When device is in sleep mode, no measurements are performed
  BMP280_MODE_FORCED = 0b01, // In forced mode, a single measurement is performed and then the device returns to sleep mode
  BMP280_MODE_NORMAL = 0b11 // In normal mode, continuous measurements are performed at the defined standby time
} BMP280Mode;

/*
  BMP280 Temperature and pressure oversampling modes
*/
typedef enum {
  BMP280_DISABLED = 0b0, // No measurement
  BMP280_OVERSAMPLING_X1 = 0b1, // Oversampling x1
  BMP280_OVERSAMPLING_X2 = 0b10, // Oversampling x2
  BMP280_OVERSAMPLING_X4 = 0b11, // Oversampling x4
  BMP280_OVERSAMPLING_X8 = 0b100, // Oversampling x8
  BMP280_OVERSAMPLING_X16 = 0b101 // Oversampling x16
} BMP280OversamplingMode;

/*
  BMP280 standby time in normal mode (time between end of measurement and start of next measurement)
*/
typedef enum {
    BMP280_STANDBY_500 = 0b0, // 0.5 ms
    BMP280_STANDBY_62_500 = 0b1, // 62.5 ms
    BMP280_STANDBY_125_000 = 0b10, // 125 ms
    BMP280_STANDBY_250_000 = 0b11, // 250 ms
    BMP280_STANDBY_500_000 = 0b100, // 500 ms
    BMP280_STANDBY_1_000_000 = 0b101, // 1000 ms
    BMP280_STANDBY_2_000_000 = 0b110, // 2000 ms
    BMP280_STANDBY_4_000_000 = 0b111 // 4000 ms
} BMP280StandbyTime;

/*
  BMP280 IIR filter settings which improve the readings in case of high dynamic changes mostly when measuring in-door
*/
typedef enum {
    BMP280_FILTER_OFF = 0b0, // Filter off
    BMP280_FILTER_2 = 0b1, // Filter coefficient 2
    BMP280_FILTER_4 = 0b10, // Filter coefficient 4
    BMP280_FILTER_8 = 0b11, // Filter coefficient 8
    BMP280_FILTER_16 = 0b100 // Filter coefficient 16
} BMP280Filter;

// Structures

/*
  BMP280 initialization parameters
*/
typedef struct {
    i2c_port_num_t i2c_port; // I2C port number to be used
    gpio_num_t sda_io_num; // GPIO number to be used for SDA signal
    gpio_num_t scl_io_num; // GPIO number to be used for SCL signal
    bool use_lp_i2c; // Whether to use LP I2C peripheral (if supported by the SoC)
    i2c_clock_source_t clk_source; // Clock source to be used by the I2C master bus
    bool use_primary_address; // Whether to use primary (0x76) or secondary (0x77) I2C address of the BMP280
    uint32_t i2c_master_freq_hz; // I2C master clock frequency in Hz
} bmp280_params_t;

/*
  BMP280 data structure which contains the human-readable temperature and pressure values
*/
typedef struct {
    float temperature;
    float pressure;
} bmp280_data_t;

/*
  Structure which contains the BMP280 calibration data and temperature fine.
*/
typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    int32_t temp_fine;
} bmp280_calibration_data_t;

/* Public function declarations */
esp_err_t bmp280_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, bmp280_params_t *params);
esp_err_t bmp280_deinit(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
esp_err_t bmp280_get_chip_id(i2c_master_dev_handle_t * dev_handle, uint8_t * chip_id);
esp_err_t bmp280_reset_chip(i2c_master_dev_handle_t * dev_handle);
esp_err_t bmp280_force_measurement(i2c_master_dev_handle_t * dev_handle);
esp_err_t bmp280_is_updating_nvm(i2c_master_dev_handle_t * dev_handle, bool * value);
esp_err_t bmp280_is_measuring(i2c_master_dev_handle_t * dev_handle, bool * value);
esp_err_t bmp280_wait_measurement_done(i2c_master_dev_handle_t * dev_handle);
esp_err_t bmp280_set_config_raw(i2c_master_dev_handle_t * dev_handle, uint8_t config);
esp_err_t bmp280_get_config_raw(i2c_master_dev_handle_t * dev_handle, uint8_t * config);
esp_err_t bmp280_set_config(i2c_master_dev_handle_t * dev_handle, BMP280Filter filter, BMP280StandbyTime standby);
esp_err_t bmp280_get_filter_standby(i2c_master_dev_handle_t * dev_handle, BMP280Filter * filter, BMP280StandbyTime * standby);
esp_err_t bmp280_set_mode_raw(i2c_master_dev_handle_t * dev_handle, uint8_t mode);
esp_err_t bmp280_get_mode_raw(i2c_master_dev_handle_t * dev_handle, uint8_t * mode);
esp_err_t bmp280_set_mode(i2c_master_dev_handle_t * dev_handle, BMP280OversamplingMode temperature_oversampling, BMP280OversamplingMode pressure_oversampling, BMP280Mode device_mode);
esp_err_t bmp280_get_mode(i2c_master_dev_handle_t * dev_handle, BMP280OversamplingMode * temperature_oversampling, BMP280OversamplingMode * pressure_oversampling, BMP280Mode * device_mode);
esp_err_t bmp280_read_data_raw(i2c_master_dev_handle_t * dev_handle, int32_t * raw_pressure, int32_t * raw_temperature);
int32_t bmp280_compensate_temperature(int32_t adc_temp, bmp280_calibration_data_t * data);
void bmp280_apply_temperature_fine(int32_t temperature, bmp280_calibration_data_t * data);
uint32_t bmp280_compensate_pressure(int32_t adc_pres, bmp280_calibration_data_t * data);
esp_err_t bmp280_read_calibration_data(i2c_master_dev_handle_t * dev_handle, bmp280_calibration_data_t * data);
esp_err_t bmp280_read(i2c_master_dev_handle_t * dev_handle, bmp280_calibration_data_t * calib_data, bmp280_data_t * bmp280_data);

#endif // BMP280_H
