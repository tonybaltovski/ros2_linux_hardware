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

#include <cstring>
#include <iostream>

#include "linux_i2c_devices/pca9685.hpp"

namespace linux_i2c_devices
{

Pca9685::Pca9685(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(i2c_interface), device_id_(device_id)
{
  i2c_interface_->open_bus();
  i2c_interface_->set_device_id(device_id_);
}

int Pca9685::initialize() {}

void Pca9685::stop() { i2c_interface_->close_bus(); }

}  // namespace linux_i2c_devices
