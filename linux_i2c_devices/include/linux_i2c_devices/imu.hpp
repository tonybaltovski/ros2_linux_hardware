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
 * @file imu.hpp
 * @brief Abstract base class for I2C inertial measurement units (accel + gyro).
 */

#pragma once

#include <cstdint>

namespace linux_i2c_devices
{

/**
 * @class Imu
 * @brief Common interface for I2C inertial measurement units providing at
 *        least 3-axis acceleration and 3-axis angular velocity
 *        (e.g. BMI160, MPU6050, ICM-20602, LSM6DS3).
 *
 * Units are sensor-native:
 *   - linear acceleration in g  (1 g = 9.80665 m/s^2)
 *   - angular velocity   in dps (degrees per second)
 *   - temperature        in degrees Celsius
 *
 * Callers that need SI units (m/s^2, rad/s) should convert at the
 * publish boundary.  Keeping the driver in sensor-native units avoids
 * lossy double conversion in the hot path and matches what the datasheet
 * specifies.
 *
 * All @c int-returning methods return 0 on success and a negative value on
 * failure (transfer error, unsupported on this device, or not yet initialised).
 */
class Imu
{
public:
  virtual ~Imu() = default;

  /// @brief Run the power-on / configuration sequence.
  virtual int initialize() = 0;

  /// @brief Put the device into a safe low-power / shutdown state.
  virtual int stop() = 0;

  /**
   * @brief Read the linear acceleration vector in the sensor body frame.
   * @param[out] x_g Acceleration along sensor X axis, g.
   * @param[out] y_g Acceleration along sensor Y axis, g.
   * @param[out] z_g Acceleration along sensor Z axis, g.
   * @return 0 on success, negative on failure.
   */
  virtual int read_accel_g(double & x_g, double & y_g, double & z_g) = 0;

  /**
   * @brief Read the angular-velocity vector in the sensor body frame.
   * @param[out] x_dps Angular rate about sensor X axis, degrees/s.
   * @param[out] y_dps Angular rate about sensor Y axis, degrees/s.
   * @param[out] z_dps Angular rate about sensor Z axis, degrees/s.
   * @return 0 on success, negative on failure.
   */
  virtual int read_gyro_dps(double & x_dps, double & y_dps, double & z_dps) = 0;

  /**
   * @brief Read the on-chip die temperature, if available.
   * @param[out] temperature_c Temperature in degrees Celsius.
   * @return 0 on success, -1 if unsupported or on transfer failure.
   *
   * Default implementation reports "unsupported".
   */
  virtual int read_temperature_c(double & temperature_c)
  {
    (void)temperature_c;
    return -1;
  }
};

}  // namespace linux_i2c_devices
