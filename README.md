# BMI055 C++ Library

[![License](https://img.shields.io/github/license/tritri0903/BMI055)](LICENSE)
[![Latest Release](https://img.shields.io/github/v/release/tritri0903/BMI055?include_prereleases)](https://github.com/tritri0903/BMI055/releases/latest)
[![Issues](https://img.shields.io/github/issues-raw/tritri0903/BMI055)](https://github.com/tritri0903/BMI055/issues)
[![Stars](https://img.shields.io/github/stars/tritri0903/BMI055)](https://github.com/tritri0903/BMI055/stargazers)

Welcome to the BMI055 C++ Library, a tool for interfacing with the BMI055 sensor. The BMI055 combines an accelerometer and a gyroscope, and this library simplifies communication, data reading, and calibration. It provides support for SPI (and later I2C interfaces).

## Overview

The BMI055 sensor is a versatile device used in various applications, including robotics, drones, and motion tracking. This library offers an easy way to harness its capabilities within your C++ projects.

## Features

- **Initialization**: Easily initialize the BMI055 sensor for quick integration.
- **Data Reading**: Retrieve accelerometer and gyroscope data with minimal effort.
- **Calibration**: Perform sensor calibration to ensure accurate readings.

## Installation

To incorporate this library into your project, simply clone the repository:

```bash
git clone https://github.com/tritri0903/BMI055.git
```

## Usage

Using the BMI055 C++ Library is straightforward:

1. Include the library's header files.
2. Create instances of the `BMI055` class for both the accelerometer and gyroscope.
3. Utilize the library's functions to initialize the sensors, read data from registers, and perform calibration.
4. Begin your application code using the data collected.

```cpp
#include "BMI055.h"

// Create instances for accelerometer and gyroscope
BMI055 accel;
BMI055 gyro;

void setup() {
  // Initialize the sensors
  accel.initialize(SPI_CLK, SPI_MOSI, SPI_MISO, SPI_ACC_NSC, SPI_CLK_FREQ);
  gyro.initialize(SPI_CLK, SPI_MOSI, SPI_MISO, SPI_ACC_NSC, SPI_CLK_FREQ);
}

void loop() {
  // Read accelerometer and gyroscope data
  
  Still in work
  
  // Perform your desired operations with the data
}
```

## Documentation

For comprehensive documentation detailing the library's functions and classes, please refer to the [API documentation](#) (documentation not available yet).
