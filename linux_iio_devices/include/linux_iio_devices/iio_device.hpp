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
 * @file iio_device.hpp
 * @brief Small helpers for reading sensors via the Linux Industrial I/O (IIO) sysfs ABI.
 *
 * The kernel exposes each IIO sensor as a directory under
 * /sys/bus/iio/devices/iio:deviceN/ with files like in_accel_x_raw,
 * in_accel_scale, sampling_frequency, trigger/current_trigger, and a
 * character device /dev/iio:deviceN for buffered reads.
 *
 * This header provides:
 *   - resolve_by_name():  find iio:deviceN by the chip's reported name.
 *   - read_double()/read_int()/read_string(): parse a single sysfs attribute.
 *   - write_string(): set an attribute (sampling frequency, scale, trigger).
 *   - BufferedReader: open /dev/iio:deviceN and poll() for hardware-triggered
 *     scan-element bursts.
 *
 * Nothing here is BMI160-specific; any IIO device that follows the standard
 * ABI works.  See https://docs.kernel.org/driver-api/iio/index.html.
 */

#pragma once

#include <poll.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace linux_iio_devices
{

/**
 * @brief Resolve an IIO device path by the kernel-reported chip name.
 * @param name Contents expected in /sys/bus/iio/devices/iio:deviceN/name
 *             (e.g. "bmi160", "bmp280", "ak09916").
 * @return Absolute path to iio:deviceN (e.g. "/sys/bus/iio/devices/iio:device0")
 *         on success, std::nullopt if no matching device is bound.
 */
std::optional<std::string> resolve_by_name(const std::string & name);

/// @brief Read a sysfs attribute as a double.  Returns nullopt on parse/IO failure.
std::optional<double> read_double(const std::string & path);

/// @brief Read a sysfs attribute as a signed integer.  Returns nullopt on parse/IO failure.
std::optional<int64_t> read_int(const std::string & path);

/// @brief Read a sysfs attribute as a trimmed string.  Returns nullopt on IO failure.
std::optional<std::string> read_string(const std::string & path);

/// @brief Write a sysfs attribute.  Returns true on success.
bool write_string(const std::string & path, const std::string & value);

/**
 * @class BufferedReader
 * @brief Open /dev/iio:deviceN and read fixed-size scan records.
 *
 * Usage: enable the desired scan_elements/<name>_en, set buffer/length and a
 * trigger via the helpers above, set buffer/enable=1, then construct this
 * and call wait_record() in a loop.
 *
 * @note The scan-record layout (offset, type, endianness, sign-extension) is
 *       per device and must be discovered from
 * scan_elements/in_<chan>_<idx>{type,index} by the caller.
 */
class BufferedReader
{
public:
  /**
   * @param device_path Absolute path to the sysfs device dir
   *                    (e.g. "/sys/bus/iio/devices/iio:device0").
   *                    The matching /dev/iio:deviceN is opened.
   * @param record_size_bytes Size of one scan record (sum of enabled scan-element
   *                          storagebits, padded to alignment, +8 if the timestamp
   *                          channel is enabled).
   * @throws std::runtime_error if /dev/iio:deviceN cannot be opened.
   */
  BufferedReader(const std::string & device_path, std::size_t record_size_bytes);
  ~BufferedReader();

  BufferedReader(const BufferedReader &) = delete;
  BufferedReader & operator=(const BufferedReader &) = delete;

  /**
   * @brief Block until one record is available or @p timeout_ms elapses.
   * @param out Filled with exactly @c record_size_bytes() on success.
   * @param timeout_ms -1 for infinite, 0 for non-blocking.
   * @return true on success, false on timeout or short read.
   */
  bool wait_record(std::vector<uint8_t> & out, int timeout_ms);

  /// @brief Return the underlying file descriptor (e.g. to integrate with rclcpp::WaitSet).
  int fd() const { return fd_; }

  /// @brief Size of one scan record in bytes.
  std::size_t record_size() const { return record_size_; }

private:
  int fd_{-1};
  std::size_t record_size_;
};

}  // namespace linux_iio_devices
