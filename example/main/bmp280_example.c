#include <stdio.h>

#include "bmp280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP_I2C_MASTER_SDA_IO         GPIO_NUM_6          /*!< GPIO number used for I2C master data  */
#define BMP_I2C_MASTER_SCL_IO         GPIO_NUM_7          /*!< GPIO number used for I2C master clock */
#define BMP_I2C_MASTER_NUM            LP_I2C_NUM_0        /*!< I2C port number for master dev */
#define BMP_LP_I2C_SRC_DEFAULT        LP_I2C_SCLK_DEFAULT /*!< I2C clock source low power, undefine it to change to normal I2C */
#define BMP_I2C_SRC_DEFAULT           I2C_CLK_SRC_DEFAULT /*!< I2C clock source */

#define USE_FORCED_MEASUREMENTS       1 // Whether to use forced measurements (saves power) or normal mode

void app_main(void)
{
  // Initialize BMP280 sensor
  bmp280_params_t params = {
    .i2c_port = BMP_I2C_MASTER_NUM,
    .sda_io_num = BMP_I2C_MASTER_SDA_IO,
    .scl_io_num = BMP_I2C_MASTER_SCL_IO,
    .use_lp_i2c = true,
    .clk_source = BMP_LP_I2C_SRC_DEFAULT,
    .use_primary_address = true,
    .i2c_master_freq_hz = 100000, // 100 kHz
  };
  i2c_master_bus_handle_t bmp280_bus_handle;
  i2c_master_dev_handle_t bmp280_dev_handle;
  esp_err_t init_status = bmp280_init(&bmp280_bus_handle, &bmp280_dev_handle, &params);
  if (init_status != ESP_OK) {
    printf("Failed to initialize BMP280 sensor: %s\n", esp_err_to_name(init_status));
    return;
  }

  // Read BMP280 chip ID to verify communication
  uint8_t chip_id = 0;
  esp_err_t chip_id_status = bmp280_get_chip_id(&bmp280_dev_handle, &chip_id);
  if (chip_id_status != ESP_OK) {
    printf("Failed to read BMP280 chip ID: %s\n", esp_err_to_name(chip_id_status));
    return;
  }
  printf("BMP280 chip ID: 0x%02X\n", chip_id);

  // Set BMP280 configuration (filter and standby time)
  esp_err_t config_set_status = bmp280_set_config(&bmp280_dev_handle, BMP280_FILTER_4, BMP280_STANDBY_250_000);
  if (config_set_status != ESP_OK) {
    printf("Failed to set BMP280 configuration: %s\n", esp_err_to_name(config_set_status));
    return;
  }

  // Set BMP280 measurement mode (temperature, pressure oversampling and device mode)
  BMP280OversamplingMode temp_oversampling = BMP280_OVERSAMPLING_X4;
  BMP280OversamplingMode press_oversampling = BMP280_OVERSAMPLING_X4;
  // If forced measurements are enabled keep the mode set to sleep, otherwise set to normal mode (from default sleep mode)
  BMP280Mode device_mode = USE_FORCED_MEASUREMENTS ? BMP280_MODE_SLEEP : BMP280_MODE_NORMAL;
  esp_err_t meas_mode_status = bmp280_set_mode(&bmp280_dev_handle, temp_oversampling, press_oversampling, device_mode);
  if (meas_mode_status != ESP_OK) {
    printf("Failed to set BMP280 measurement mode: %s\n", esp_err_to_name(meas_mode_status));
    return;
  }

  // Load calibration data
  bmp280_calibration_data_t calib_data = {0};
  esp_err_t calib_status = bmp280_read_calibration_data(&bmp280_dev_handle, &calib_data);
  if (calib_status != ESP_OK) {
    printf("Failed to read BMP280 calibration data: %s\n", esp_err_to_name(calib_status));
    return;
  }

  // Perform 10 measurements every 5 seconds
  for (int i = 0; i < 10; ++i) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds

    if (USE_FORCED_MEASUREMENTS) { // In forced mode, trigger a measurement
      if (bmp280_force_measurement(&bmp280_dev_handle) != ESP_OK) {
        printf("Failed to force measurement on BMP280 sensor\n");
        continue;
      }
    }

    // Read raw data from BMP280 (this will also wait for measurement to complete if it's in progress)
    bmp280_data_t bmp280_data = {0};
    esp_err_t read_status = bmp280_read(&bmp280_dev_handle, &calib_data, &bmp280_data);
    if (read_status != ESP_OK) {
      printf("Failed to read data from BMP280 sensor: %s\n", esp_err_to_name(read_status));
      continue;
    }

    // Print the results
    printf("Finished measurement %d:\n", i + 1);
    printf("  Temperature: %.2f °C\n", bmp280_data.temperature);
    printf("  Pressure: %.2f hPa\n", bmp280_data.pressure / 100.0); // Convert Pa to hPa
  }

  // Reset the BMP280 chip
  esp_err_t reset_status = bmp280_reset_chip(&bmp280_dev_handle);
  if (reset_status != ESP_OK) {
    printf("Failed to reset BMP280 sensor: %s\n", esp_err_to_name(reset_status));
    return;
  }

  // Read the temperature oversampling setting after reset (should be back to default)
  BMP280OversamplingMode temp_oversampling_after_reset;
  esp_err_t oversampling_status = bmp280_get_mode(&bmp280_dev_handle, &temp_oversampling_after_reset, NULL, NULL);
  if (oversampling_status != ESP_OK) {
    printf("Failed to read temperature oversampling after reset: %s\n", esp_err_to_name(oversampling_status));
    return;
  }
  printf("Temperature oversampling after reset: %d (should be 1 for default)\n", temp_oversampling_after_reset);

  // Deinitialize BMP280 sensor
  esp_err_t deinit_status = bmp280_deinit(&bmp280_bus_handle, &bmp280_dev_handle);
  if (deinit_status != ESP_OK) {
    printf("Failed to deinitialize BMP280 sensor: %s\n", esp_err_to_name(deinit_status));
    return;
  }
}