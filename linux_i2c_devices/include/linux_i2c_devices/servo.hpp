// Copyright 2020 Tony Baltovski
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
 * @file servo.hpp
 * @brief Abstract base class for servo-style PWM-driven actuators.
 */

#pragma once

#include <cstdint>

namespace linux_i2c_devices
{

/** @name Default servo pulse-width range (microseconds). */
///@{
constexpr uint16_t SERVO_DEFAULT_MIN_PULSE_US = 1000;
constexpr uint16_t SERVO_DEFAULT_MAX_PULSE_US = 2000;
///@}

/** @name Default mechanical actuation range for set_angle_deg(). */
///@{
constexpr double SERVO_DEFAULT_ACTUATION_RANGE_DEG = 180.0;
///@}

/**
 * @class Servo
 * @brief Common interface for a single PWM-driven actuator
 *
 * Mirrors the @c Screen abstraction: callers code against this interface and a
 * concrete subclass (e.g. @c Pca9685Servo) carries the backend specifics.
 * Implementations own per-channel calibration (min / max pulse width and
 * actuation range) so high-level commands (angle / fraction / throttle) can be
 * issued without thinking in duty cycles or ticks.
 *
 * All @c int-returning methods return 0 on success and -1 on transfer or
 * argument failure (errno set by the underlying transport).
 */
class Servo
{
public:
  virtual ~Servo() = default;

  /**
   * @brief Drive the servo to @p angle_deg in [0, actuation_range_deg].
   *        Out-of-range values are clamped.
   */
  virtual int set_angle_deg(double angle_deg) = 0;

  /**
   * @brief Drive the servo with a normalised command in [0.0, 1.0].
   *        0.0 -> min pulse, 1.0 -> max pulse.  Clamped.
   */
  virtual int set_fraction(double fraction) = 0;

  /**
   * @brief Drive a continuous-rotation servo / ESC with a throttle in [-1.0, +1.0].
   *        -1.0 -> min pulse (full reverse), 0.0 -> midpoint, +1.0 -> max pulse.  Clamped.
   */
  virtual int set_throttle(double throttle) = 0;

  /**
   * @brief Drive the servo by raw pulse width in microseconds.
   *        Clamped to the configured [min_us, max_us] so an out-of-range value
   *        cannot jam the servo against its mechanical stop.
   */
  virtual int set_pulse_us(uint16_t pulse_us) = 0;

  /// @brief Release the output (drive the channel low / off).
  virtual int release() = 0;

  /// @brief Minimum pulse width in microseconds for this channel.
  virtual uint16_t min_pulse_us() const = 0;

  /// @brief Maximum pulse width in microseconds for this channel.
  virtual uint16_t max_pulse_us() const = 0;

  /// @brief Mechanical actuation range in degrees used by set_angle_deg().
  virtual double actuation_range_deg() const = 0;

  /// @brief Last commanded pulse width in microseconds (post-clamp).
  virtual uint16_t last_pulse_us() const = 0;

  /// @brief Last commanded angle in degrees, derived from the last pulse.
  virtual double last_angle_deg() const = 0;
};

}  // namespace linux_i2c_devices
