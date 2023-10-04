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
  accel.begin(SPI_CLK, SPI_MOSI, SPI_MISO, SPI_ACC_NSC, SPI_CLK_FREQ);
  gyro.begin(SPI_CLK, SPI_MOSI, SPI_MISO, SPI_ACC_NSC, SPI_CLK_FREQ);
}

void loop() {
  // Read accelerometer and gyroscope data
  
  Still in work
  
  // Perform your desired operations with the data
}
```

## Documentation

For comprehensive documentation detailing the library's functions and classes, please refer to the [API documentation](#) (documentation not available yet).

[Link](https://d1wqtxts1xzle7.cloudfront.net/56549933/a_robust_and_easy_to_implement_method_for_imu_calibration-libre.pdf?1526170383=&response-content-disposition=inline%3B+filename%3DA_Robust_and_Easy_to_Implement_Method_fo.pdf&Expires=1696416050&Signature=Hq0shx3OyRK6VLfaZVdFGqdtFtRwn-UuiLORjgDrwywz4vMlT4CPhrsQoWGtX7FKo8sVGlH311XDt1wsQZ9W3jidDavPmXZDb~bQnKH80v8KrFdb2I7EPK-TZ0mE2LUefluYd8J3ROBqOKbl0Itxv3FbCnJsH9MdU-4xe9jnYPCmHlEo12fOj13AEZphJ2AqnEPxYup4hpgvhz00KzG~fxDW9a-bP2ixY-gR31FBkNBRooFg6Z3OOFnZz0~LdNr1zfwmSDoWZHsPYIJRpQ9X1Ufu9QVBCA7bGXVufuhTyHBwBoUPvqKDchEuSA9gIWPz1JwIgAHSHC2AAGifVbLKQg__&Key-Pair-Id=APKAJLOHF5GGSLRBV4ZA) to the paper used for the calibration technique.
