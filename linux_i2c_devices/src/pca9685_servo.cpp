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

#include "linux_i2c_devices/pca9685_servo.hpp"

#include <algorithm>
#include <stdexcept>

namespace linux_i2c_devices
{

Pca9685Servo::Pca9685Servo(
  Pca9685 & pca9685, uint8_t channel, uint16_t min_pulse_us, uint16_t max_pulse_us,
  double actuation_range_deg)
: pca9685_(pca9685),
  channel_(channel),
  min_pulse_us_(min_pulse_us),
  max_pulse_us_(max_pulse_us),
  actuation_range_deg_(actuation_range_deg),
  last_pulse_us_(static_cast<uint16_t>((min_pulse_us + max_pulse_us) / 2))
{
  if (channel_ >= PCA9685_NUM_CHANNELS)
  {
    throw std::invalid_argument("Pca9685Servo: channel must be 0..15");
  }
  if (max_pulse_us_ <= min_pulse_us_)
  {
    throw std::invalid_argument("Pca9685Servo: max_pulse_us must be > min_pulse_us");
  }
  if (actuation_range_deg_ <= 0.0)
  {
    throw std::invalid_argument("Pca9685Servo: actuation_range_deg must be > 0");
  }
}

int Pca9685Servo::set_pulse_us(uint16_t pulse_us)
{
  pulse_us = std::clamp(pulse_us, min_pulse_us_, max_pulse_us_);
  // duty = pulse_seconds * frequency = pulse_us * freq * 1e-6.
  const double duty = static_cast<double>(pulse_us) * pca9685_.frequency_hz() * 1e-6;
  if (pca9685_.set_duty_cycle(channel_, duty) < 0)
  {
    return -1;
  }
  last_pulse_us_ = pulse_us;
  return 0;
}

int Pca9685Servo::set_fraction(double fraction)
{
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double span = static_cast<double>(max_pulse_us_ - min_pulse_us_);
  const auto pulse_us = static_cast<uint16_t>(min_pulse_us_ + fraction * span + 0.5);
  return set_pulse_us(pulse_us);
}

int Pca9685Servo::set_throttle(double throttle)
{
  throttle = std::clamp(throttle, -1.0, 1.0);
  return set_fraction((throttle + 1.0) * 0.5);
}

int Pca9685Servo::set_angle_deg(double angle_deg)
{
  angle_deg = std::clamp(angle_deg, 0.0, actuation_range_deg_);
  return set_fraction(angle_deg / actuation_range_deg_);
}

int Pca9685Servo::release()
{
  // PCA9685 special: on=0, off=4096 forces the channel fully off.
  return pca9685_.set_pwm(channel_, 0, PCA9685_MAX_COUNT);
}

double Pca9685Servo::last_angle_deg() const
{
  const double span = static_cast<double>(max_pulse_us_ - min_pulse_us_);
  const double fraction = static_cast<double>(last_pulse_us_ - min_pulse_us_) / span;
  return fraction * actuation_range_deg_;
}

}  // namespace linux_i2c_devices
