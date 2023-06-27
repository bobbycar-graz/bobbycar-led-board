#pragma once

// system includes
#include <cstdint>

using pin_t = uint8_t;

// MOSFETs
constexpr pin_t MOSFET_1 = 13;
constexpr pin_t MOSFET_2 = 12;

// WS2812B
constexpr pin_t WS2812B_DATA_PIN = 26;

// Sensors
constexpr pin_t BAT_VOLTAGE_SENSOR = 32;
constexpr float BAT_VOLTAGE_RATIO = 22.604; // 22k + 1k voltage divider

constexpr pin_t V5V_VOLTAGE_SENSOR = 33;
constexpr float V5V_VOLTAGE_RATIO = 2; // 1k + 1k voltage divider

// I2C
constexpr pin_t I2C_SDA = 16;
constexpr pin_t I2C_SCL = 17;

// Current Sensor
constexpr pin_t CURRENT_SENSOR = 35;
constexpr int CURRENT_SENSOR_RATIO = 145; // calibration  (A/adc_value)
