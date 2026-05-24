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
 * @file magnetometer.hpp
 * @brief Abstract base class for I2C magnetometer / compass devices.
 */

#pragma once

#include <cstdint>

namespace linux_i2c_devices
{

/**
 * @class Magnetometer
 * @brief Common interface for I2C magnetometers and electronic compasses
 *        (e.g. HMC5883L, HMC6343, QMC5883, LIS3MDL).
 *
 * Callers code against this interface; concrete subclasses carry the
 * device-specific register map and command set.  Units are SI:
 *   - magnetic field in microtesla (uT)
 *   - heading / pitch / roll in degrees
 *
 * Not every device measures every quantity natively.  Subclasses that cannot
 * provide a given reading should return -1 from the corresponding method and
 * leave the output arguments untouched, so that callers can detect the
 * capability at runtime.
 *
 * All @c int-returning methods return 0 on success and a negative value on
 * failure (transfer error, unsupported on this device, or not yet initialised).
 */
class Magnetometer
{
public:
  virtual ~Magnetometer() = default;

  /// @brief Run the power-on / configuration sequence.
  virtual int initialize() = 0;

  /// @brief Put the device into a safe low-power / shutdown state.
  virtual int stop() = 0;

  /**
   * @brief Read the raw magnetic-field vector in the sensor body frame.
   * @param[out] x_ut Field along sensor X axis, microtesla.
   * @param[out] y_ut Field along sensor Y axis, microtesla.
   * @param[out] z_ut Field along sensor Z axis, microtesla.
   * @return 0 on success, negative on failure.
   */
  virtual int read_field_ut(double & x_ut, double & y_ut, double & z_ut) = 0;

  /**
   * @brief Read a tilt-compensated heading, if the device provides one.
   * @param[out] heading_deg Heading in degrees, [0, 360), 0 = sensor +X axis.
   * @return 0 on success, -1 if unsupported or on transfer failure.
   *
   * Default implementation reports "unsupported".  Devices with on-chip
   * tilt-comp (e.g. HMC6343) should override.
   */
  virtual int read_heading_deg(double & heading_deg)
  {
    (void)heading_deg;
    return -1;
  }

  /**
   * @brief Read pitch and roll from an on-chip accelerometer, if present.
   * @param[out] pitch_deg Pitch in degrees.
   * @param[out] roll_deg  Roll in degrees.
   * @return 0 on success, -1 if unsupported or on transfer failure.
   */
  virtual int read_tilt_deg(double & pitch_deg, double & roll_deg)
  {
    (void)pitch_deg;
    (void)roll_deg;
    return -1;
  }
};

}  // namespace linux_i2c_devices
