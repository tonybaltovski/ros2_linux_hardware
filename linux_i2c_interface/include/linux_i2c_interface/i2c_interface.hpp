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
 * @file i2c_interface.hpp
 * @brief Thread-safe Linux I2C bus interface.
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace linux_i2c_interface
{

/**
 * @class I2cInterface
 * @brief Provides thread-safe read/write access to a Linux I2C bus.
 *
 * All public bus operations are serialised with an internal mutex so that a
 * single I2cInterface instance can be shared safely between threads.
 */
class I2cInterface
{
public:
  /**
   * @brief Construct an I2C interface from a full device path.
   * @param i2c_bus Device path, e.g. "/dev/i2c-1".
   */
  explicit I2cInterface(const std::string & i2c_bus);

  /**
   * @brief Construct an I2C interface from a bus number.
   * @param i2c_bus_number Bus index (e.g. 1 → "/dev/i2c-1").
   */
  explicit I2cInterface(uint8_t i2c_bus_number);

  /**
   * @brief Open the I2C bus file descriptor.
   * @return 0 on success, -1 on failure.
   */
  int8_t open_bus();

  /**
   * @brief Close the I2C bus file descriptor.
   * @return 0 on success, negative value on failure.
   */
  int8_t close_bus();

  /**
   * @brief Check whether the bus is currently open.
   * @return true if connected.
   */
  bool is_connected() const;

  /**
   * @brief Select the active I2C slave device.
   * @param device_id 7-bit I2C slave address.
   * @return 0 on success, negative value on failure.
   */
  int8_t set_device_id(uint8_t device_id);

  /**
   * @brief Read bytes from the currently selected device.
   * @param address Register address to read from.
   * @param[out] data Buffer to receive the data.
   * @param count Number of bytes to read.
   * @return 0 on success, -1 on failure.
   */
  int8_t read_from_bus(uint8_t address, void * data, uint32_t count);

  /**
   * @brief Write a single command byte (no payload) to the bus.
   * @param address Command/register byte to send.
   * @return 0 on success, -1 on failure.
   */
  int8_t write_to_bus(uint8_t address);

  /**
   * @brief Select a device and write a single command byte.
   * @param device_id 7-bit I2C slave address.
   * @param address Command/register byte to send.
   * @return 0 on success, negative value on failure.
   */
  int8_t write_to_bus(uint8_t device_id, uint8_t address);

  /**
   * @brief Write a register address followed by a data payload.
   * @param address Register address byte.
   * @param data Pointer to the payload bytes.
   * @param count Number of payload bytes to write.
   * @return 0 on success, -1 on failure.
   */
  int8_t write_to_bus(uint8_t address, void * data, uint32_t count);

  /**
   * @brief Write a raw byte buffer as a single I2C transaction.
   *
   * Unlike write_to_bus(), the entire buffer (including any control or
   * register bytes) is sent in one write() call so that the device sees a
   * single START … STOP sequence.  This is required by devices such as the
   * SSD1306 OLED controller.
   *
   * @param data  Pointer to the complete byte sequence to send.
   * @param count Number of bytes.
   * @return 0 on success, -1 on failure.
   */
  int8_t write_raw(const void * data, uint32_t count);

private:
  std::string i2c_bus_;       ///< Device path (e.g. "/dev/i2c-1").
  int i2c_fd_{-1};            ///< File descriptor for the open bus.
  std::mutex i2c_mutex_;      ///< Serialises all bus operations.
  bool is_connected_{false};  ///< Whether the bus is currently open.

  /**
   * @brief Internal helper to close the bus without locking the mutex.
   * @return 0 on success, negative value on failure.
   *
   * Must only be called while i2c_mutex_ is already held.
   */
  int8_t close_bus_unlocked();
};

}  // namespace linux_i2c_interface
