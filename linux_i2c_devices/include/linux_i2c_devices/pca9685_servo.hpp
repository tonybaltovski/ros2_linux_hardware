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
 * @file pca9685_servo.hpp
 * @brief Servo wrapper around a single PCA9685 channel.
 */

#pragma once

#include <cstdint>

#include "linux_i2c_devices/pca9685.hpp"
#include "linux_i2c_devices/servo.hpp"

namespace linux_i2c_devices
{

/**
 * @class Pca9685Servo
 * @brief @c Servo implementation backed by one channel of a @c Pca9685.
 *
 * The Pca9685 driver stays chip-shaped (ticks / duty cycle / frequency).
 * This wrapper holds per-channel calibration (min/max pulse width, actuation
 * range) and converts angle / fraction / throttle commands into duty cycles.
 *
 * The PWM frequency comes from the underlying @c Pca9685; set it once on the
 * chip (typically 50 Hz for hobby servos and ESCs) before driving the servo.
 *
 * Typical usage:
 * @code
 * auto i2c = linux_i2c_interface::I2cInterface::get_shared(1);
 * linux_i2c_devices::Pca9685 pwm(i2c, 0x40);
 * pwm.initialize();
 * pwm.set_pwm_frequency(50.0);
 *
 * linux_i2c_devices::Pca9685Servo steering(pwm, 0);                // ch0, 1.0..2.0 ms
 * linux_i2c_devices::Pca9685Servo throttle(pwm, 1, 1100, 1900);    // ch1, ESC range
 *
 * steering.set_angle_deg(90.0);   // centre
 * throttle.set_throttle(0.25);    // 25% forward
 * @endcode
 *
 * Holds a reference to the @c Pca9685; the chip must outlive every servo that
 * wraps it.  Not thread-safe on its own, but underlying I2C calls are
 * serialised by the bus mutex.
 */
class Pca9685Servo : public Servo
{
public:
  /**
   * @brief Construct a Pca9685Servo on @p channel of @p pca9685.
   * @param pca9685 Underlying PCA9685 (must outlive this servo).
   * @param channel 0..15.
   * @param min_pulse_us Pulse width at the minimum command.
   * @param max_pulse_us Pulse width at the maximum command.  Must be > @p min_pulse_us.
   * @param actuation_range_deg Mechanical range used by set_angle_deg().
   * @throws std::invalid_argument if @p channel is out of range, @p max_pulse_us
   *         is not strictly greater than @p min_pulse_us, or
   *         @p actuation_range_deg is not positive.
   */
  Pca9685Servo(
    Pca9685 & pca9685, uint8_t channel, uint16_t min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US,
    uint16_t max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US,
    double actuation_range_deg = SERVO_DEFAULT_ACTUATION_RANGE_DEG);

  /// @copydoc Servo::set_angle_deg
  int set_angle_deg(double angle_deg) override;
  /// @copydoc Servo::set_fraction
  int set_fraction(double fraction) override;
  /// @copydoc Servo::set_throttle
  int set_throttle(double throttle) override;
  /// @copydoc Servo::set_pulse_us
  int set_pulse_us(uint16_t pulse_us) override;
  /// @copydoc Servo::release
  int release() override;

  /// @copydoc Servo::min_pulse_us
  uint16_t min_pulse_us() const override { return min_pulse_us_; }
  /// @copydoc Servo::max_pulse_us
  uint16_t max_pulse_us() const override { return max_pulse_us_; }
  /// @copydoc Servo::actuation_range_deg
  double actuation_range_deg() const override { return actuation_range_deg_; }
  /// @copydoc Servo::last_pulse_us
  uint16_t last_pulse_us() const override { return last_pulse_us_; }
  /// @copydoc Servo::last_angle_deg
  double last_angle_deg() const override;

  /// @brief Channel this servo is bound to (0..15).
  uint8_t channel() const { return channel_; }

private:
  Pca9685 & pca9685_;
  uint8_t channel_;
  uint16_t min_pulse_us_;
  uint16_t max_pulse_us_;
  double actuation_range_deg_;
  // The wrapper itself is not thread-safe (see class docstring); the
  // underlying Pca9685 serialises the actual I2C write.
  uint16_t last_pulse_us_;
};

}  // namespace linux_i2c_devices
