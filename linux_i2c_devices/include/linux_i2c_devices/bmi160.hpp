// Copyright 2026 Tony Baltovski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file bmi160.hpp
 * @brief Driver for the Bosch BMI160 6-axis IMU (3-axis accel + 3-axis gyro).
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "linux_i2c_devices/imu.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_devices
{

/** @name BMI160 default I2C addresses (7-bit). */
///@{
constexpr uint8_t BMI160_ADDRESS_SDO_LOW = 0x68;   ///< Default; SDO pin tied to GND.
constexpr uint8_t BMI160_ADDRESS_SDO_HIGH = 0x69;  ///< SDO pin tied to V_DDIO.
constexpr uint8_t BMI160_DEFAULT_ADDRESS = BMI160_ADDRESS_SDO_LOW;
constexpr uint8_t BMI160_CHIP_ID_VALUE = 0xD1;
///@}

/** @name BMI160 register map (subset used by this driver). */
///@{
constexpr uint8_t BMI160_REG_CHIP_ID = 0x00;
constexpr uint8_t BMI160_REG_ERR = 0x02;
constexpr uint8_t BMI160_REG_PMU_STATUS = 0x03;
constexpr uint8_t BMI160_REG_DATA_GYRO_X_LSB = 0x0C;  ///< 12 bytes contiguous: gyro X/Y/Z, accel X/Y/Z (little-endian int16).
constexpr uint8_t BMI160_REG_TEMPERATURE_LSB = 0x20;  ///< 2 bytes (little-endian int16).
constexpr uint8_t BMI160_REG_ACC_CONF = 0x40;
constexpr uint8_t BMI160_REG_ACC_RANGE = 0x41;
constexpr uint8_t BMI160_REG_GYR_CONF = 0x42;
constexpr uint8_t BMI160_REG_GYR_RANGE = 0x43;
constexpr uint8_t BMI160_REG_CMD = 0x7E;
///@}

/** @name BMI160 CMD register values. */
///@{
constexpr uint8_t BMI160_CMD_ACC_NORMAL = 0x11;  ///< Set accel PMU to normal.
constexpr uint8_t BMI160_CMD_GYR_NORMAL = 0x15;  ///< Set gyro  PMU to normal.
constexpr uint8_t BMI160_CMD_ACC_SUSPEND = 0x10;
constexpr uint8_t BMI160_CMD_GYR_SUSPEND = 0x14;
constexpr uint8_t BMI160_CMD_SOFT_RESET = 0xB6;
///@}

/** @name PMU wake-up / settling times (microseconds), per datasheet. */
///@{
constexpr uint32_t BMI160_SOFT_RESET_DELAY_US = 15000;  ///< 15 ms after softreset.
constexpr uint32_t BMI160_ACC_WAKEUP_DELAY_US = 4000;   ///< 3.8 ms typical.
constexpr uint32_t BMI160_GYR_WAKEUP_DELAY_US = 80000;  ///< 80 ms typical.
///@}

/**
 * @enum AccelRange
 * @brief Full-scale range selection for the accelerometer.
 */
enum class Bmi160AccelRange : uint8_t
{
  RANGE_2G = 0x03,   ///< ±2 g  (default; highest resolution).
  RANGE_4G = 0x05,
  RANGE_8G = 0x08,
  RANGE_16G = 0x0C,  ///< ±16 g (lowest resolution).
};

/**
 * @enum GyroRange
 * @brief Full-scale range selection for the gyroscope.
 */
enum class Bmi160GyroRange : uint8_t
{
  RANGE_2000_DPS = 0x00,  ///< ±2000 dps (default).
  RANGE_1000_DPS = 0x01,
  RANGE_500_DPS = 0x02,
  RANGE_250_DPS = 0x03,
  RANGE_125_DPS = 0x04,   ///< ±125 dps (highest resolution).
};

/**
 * @class Bmi160
 * @brief Driver for the Bosch BMI160 6-axis IMU.
 *
 * After reset both sub-blocks are in suspend mode and consume <5 µA.
 * @ref initialize() issues a soft reset, brings the accelerometer and
 * gyroscope to normal power mode, and applies the requested full-scale
 * ranges.  Output data rate defaults to the post-reset value of 100 Hz on
 * both blocks; tune by writing @ref BMI160_REG_ACC_CONF / @ref
 * BMI160_REG_GYR_CONF if needed.
 *
 * All @c int-returning methods return 0 on success, -1 on failure.
 */
class Bmi160 : public Imu
{
public:
  /**
   * @brief Construct a Bmi160 driver.
   * @param i2c_interface Shared I2C bus interface (must not be null).
   * @param device_id 7-bit I2C address (default 0x68; 0x69 if SDO=V_DDIO).
   * @param accel_range Accelerometer full-scale range (default ±2 g).
   * @param gyro_range  Gyroscope full-scale range     (default ±2000 dps).
   * @throws std::invalid_argument if @p i2c_interface is null.
   */
  Bmi160(
    std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface,
    uint8_t device_id = BMI160_DEFAULT_ADDRESS,
    Bmi160AccelRange accel_range = Bmi160AccelRange::RANGE_2G,
    Bmi160GyroRange gyro_range = Bmi160GyroRange::RANGE_2000_DPS);

  /// @copydoc Imu::initialize
  int initialize() override;
  /// @copydoc Imu::stop
  int stop() override;
  /// @copydoc Imu::read_accel_g
  int read_accel_g(double & x_g, double & y_g, double & z_g) override;
  /// @copydoc Imu::read_gyro_dps
  int read_gyro_dps(double & x_dps, double & y_dps, double & z_dps) override;
  /// @copydoc Imu::read_temperature_c
  int read_temperature_c(double & temperature_c) override;

  /**
   * @brief Read accelerometer and gyroscope in a single burst transfer.
   *        Cheaper than calling read_accel_g + read_gyro_dps separately.
   */
  int read_imu(
    double & ax_g, double & ay_g, double & az_g,
    double & gx_dps, double & gy_dps, double & gz_dps);

  /**
   * @brief Read raw 16-bit accel + gyro counts in the chip's native units.
   *        Useful if you want to apply your own calibration / scale.
   */
  int read_imu_raw(
    int16_t & ax, int16_t & ay, int16_t & az,
    int16_t & gx, int16_t & gy, int16_t & gz);

private:
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  Bmi160AccelRange accel_range_;
  Bmi160GyroRange gyro_range_;
  double accel_lsb_to_g_;
  double gyro_lsb_to_dps_;
  std::string log_name_;
  bool initialized_;
};

}  // namespace linux_i2c_devices
