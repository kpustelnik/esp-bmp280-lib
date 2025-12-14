# BMP280 ESP-IDF Library

A comprehensive ESP-IDF component library for the Bosch BMP280 digital pressure and temperature sensor with I2C communication support.
Please see the [sensor documentation](./docs/bst-bmp280-ds001.pdf) for further usage details.

## Features

- **Easy Integration**: Simple API for ESP-IDF projects
- **I2C Communication**: Full support for I2C master interface
- **Flexible Configuration**: Support for all BMP280 operating modes and settings
- **Low Power Options**: LP I2C peripheral support (for compatible SoCs)
- **Calibration Support**: Automatic calibration data handling
- **Multiple Modes**: Sleep, forced, and normal measurement modes
- **Configurable Oversampling**: Temperature and pressure oversampling options (x1 to x16)
- **IIR Filtering**: Built-in digital filter support
- **Dual Address Support**: Both primary (0x76) and secondary (0x77) I2C addresses

## Installation

### Using ESP Component Registry

Add to your project's `idf_component.yml`:

```yaml
dependencies:
  kpustelnik/esp-bmp280-lib:
    version: "^1.0.0"
    path: ...
```
NOTE: Path to the library has to be specified since the component is not published to the ESP IDF repository.

### Manual Installation

Clone this repository into your project's `components` directory:

```bash
cd your_project/components
git clone https://github.com/kpustelnik/esp-bmp280-lib.git
```

You may also want to use git submodules to keep track of the library version.

## Quick Start

Minimal setup
```c
#include "bmp280.h"

void app_main(void) {
    // Initialize BMP280 sensor
    bmp280_params_t params = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .use_lp_i2c = false,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .use_primary_address = true,
        .i2c_master_freq_hz = BMP280_I2C_MASTER_FREQ_HZ,
    };
    
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    bmp280_init(&bus_handle, &dev_handle, &params);
    
    // Configure sensor
    bmp280_set_config(&dev_handle, BMP280_FILTER_4, BMP280_STANDBY_250_000);
    bmp280_set_mode(&dev_handle, BMP280_OVERSAMPLING_X4, BMP280_OVERSAMPLING_X4, BMP280_MODE_NORMAL);
    
    // Read calibration data
    bmp280_calibration_data_t calib_data;
    bmp280_read_calibration_data(&dev_handle, &calib_data);
    
    // Read sensor data
    bmp280_data_t sensor_data;
    bmp280_read(&dev_handle, &calib_data, &sensor_data);
    
    printf("Temperature: %.2f °C\n", sensor_data.temperature);
    printf("Pressure: %.2f hPa\n", sensor_data.pressure / 100.0);
    
    // Cleanup
    bmp280_deinit(&bus_handle, &dev_handle);
}
```
One may want to add error checking - please check out the [example](./example/main/bmp280_example.c) for more detailed usage.

## API Reference

### Initialization & Configuration

- `bmp280_init()` - Initialize I2C bus and BMP280 device
- `bmp280_deinit()` - Deinitialize I2C bus and device
- `bmp280_get_chip_id()` - Read BMP280 chip ID (should return 0x58)
- `bmp280_reset_chip()` - Perform soft reset of the sensor

### Operating Modes

- `bmp280_set_mode()` - Set temperature/pressure oversampling and device mode
- `bmp280_get_mode()` - Read current mode settings
- `bmp280_set_mode_raw()` - Set raw mode register value
- `bmp280_get_mode_raw()` - Read raw mode register value

**Supported Modes:**
- `BMP280_MODE_SLEEP` - No measurements (lowest power)
- `BMP280_MODE_FORCED` - Single measurement then return to sleep
- `BMP280_MODE_NORMAL` - Continuous measurements

**Oversampling Options:**
- `BMP280_DISABLED` - Measurement disabled
- `BMP280_OVERSAMPLING_X1`, `BMP280_OVERSAMPLING_X2`, `BMP280_OVERSAMPLING_X4`, `BMP280_OVERSAMPLING_X8`, `BMP280_OVERSAMPLING_X16` - Various oversampling rates

### Filter & Timing Configuration

- `bmp280_set_config()` - Set IIR filter and standby time
- `bmp280_get_config()` - Read filter and standby settings
- `bmp280_set_config_raw()` - Set raw config register value
- `bmp280_get_config_raw()` - Read raw config register value

**Filter Coefficients:**
- `BMP280_FILTER_OFF`, `BMP280_FILTER_2`, `BMP280_FILTER_4`, `BMP280_FILTER_8`, `BMP280_FILTER_16`

**Standby Times:**
- `BMP280_STANDBY_500` (0.5 ms)
- `BMP280_STANDBY_62_500` (62.5 ms)
- `BMP280_STANDBY_125_000` (125 ms)
- `BMP280_STANDBY_250_000` (250 ms)
- `BMP280_STANDBY_500_000` (500 ms)
- `BMP280_STANDBY_1_000_000` (1000 ms)
- `BMP280_STANDBY_2_000_000` (2000 ms)
- `BMP280_STANDBY_4_000_000` (4000 ms)

### Measurement Functions

- `bmp280_force_measurement()` - Trigger single measurement in forced mode
- `bmp280_is_measuring()` - Check if measurement is in progress
- `bmp280_wait_measurement_done()` - Wait synchronously for measurement completion
- `bmp280_is_updating_nvm()` - Check if NVM is being copied to registers
- `bmp280_read_data_raw()` - Read raw ADC values
- `bmp280_read()` - Read compensated temperature and pressure values

### Calibration & Compensation

- `bmp280_read_calibration_data()` - Read factory calibration coefficients
- `bmp280_compensate_temperature()` - Apply temperature compensation formula
- `bmp280_compensate_pressure()` - Apply pressure compensation formula
- `bmp280_apply_temperature_fine()` - Set temperature fine value manually

## Example

A complete working example is available in the [`example`](example/) directory. The example demonstrates:

- Sensor initialization with I2C configuration
- Chip ID verification
- Configuration of filter and standby time
- Setting measurement modes and oversampling
- Reading calibration data
- Performing periodic measurements (either forced or normal modes)
- Data conversion and display
- Sensor reset functionality
- Proper cleanup and deinitialization

To run the example after cloning the repository:

```bash
cd example
idf.py set-target esp32  # or your target chip
idf.py build
idf.py flash monitor
```

## Configuration

Key parameters in `bmp280_params_t`:

- `i2c_port` - I2C port (controller) number (I2C_NUM_0, I2C_NUM_1, etc.)
- `sda_io_num` - GPIO pin for SDA
- `scl_io_num` - GPIO pin for SCL
- `use_lp_i2c` - Use low-power I2C peripheral (if available)
- `clk_source` - I2C clock source selection (normal or low power one)
- `use_primary_address` - true for 0x76, false for 0x77 (based on how the chip is powered)
- `i2c_master_freq_hz` - I2C clock frequency (recommended: 100 kHz)

## References

- [BMP280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)