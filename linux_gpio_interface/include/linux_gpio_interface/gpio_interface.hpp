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
 * @file gpio_interface.hpp
 * @brief Thread-safe Linux GPIO character-device interface.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace linux_gpio_interface
{

/// @brief Direction of a requested GPIO line.
enum class GpioDirection
{
  Input,
  Output,
};

/// @brief Internal pull bias applied to a line. Not all hardware supports all values.
enum class GpioBias
{
  AsIs,      ///< Leave whatever the kernel has configured.
  Disabled,  ///< Explicitly disable internal pull.
  PullUp,
  PullDown,
};

/// @brief Output driver mode.
enum class GpioDrive
{
  PushPull,
  OpenDrain,
  OpenSource,
};

/**
 * @class GpioInterface
 * @brief Provides access to GPIO lines on a single `/dev/gpiochipN` device.
 *
 * Uses the Linux GPIO character-device v2 ABI (`<linux/gpio.h>`) directly,
 * so no external library (libgpiod, WiringPi, sysfs) is required.  The
 * deprecated sysfs interface is not used.
 *
 * The chip's `/dev/gpiochipN` fd is owned by the interface and opened lazily
 * on the first line request.  Individual lines, once requested, own their own
 * fd (returned by the kernel) and remain valid even if the interface is later
 * destroyed.  In practice you should keep both alive — use `get_shared()` to
 * obtain a single process-wide instance per chip.
 *
 * **Error reporting.** All methods return 0 / a valid object on success and
 * -1 / `!ok()` on failure.  On failure, `errno` carries the cause; common
 * values include `ENOENT` (chip missing), `EACCES` (no group permission),
 * `EBUSY` (line already requested by another process), and `EINVAL`
 * (unsupported flag combination on the running kernel).
 *
 * **Threading.** Requesting lines is serialised by the chip mutex.  Once a
 * `Line` is requested, its `set_value()` / `get_value()` calls are
 * independent of the chip and may run concurrently from any thread without
 * additional locking.
 */
class GpioInterface
{
public:
  /**
   * @class Line
   * @brief RAII handle to one or more requested GPIO lines.
   *
   * Owns a kernel-issued line-request fd; releases the line(s) on destruction.
   * Move-only.  A default-constructed `Line` is `!ok()` and ignores all calls.
   */
  class Line
  {
  public:
    Line() = default;
    Line(const Line &) = delete;
    Line & operator=(const Line &) = delete;
    Line(Line && other) noexcept;
    Line & operator=(Line && other) noexcept;
    ~Line();

    /// @brief True when the kernel granted the line request.
    bool ok() const { return fd_ >= 0; }

    /// @brief Number of lines bundled into this request.
    uint32_t size() const { return num_lines_; }

    /// @brief Set the value of a single-line request (0 or 1).
    int set_value(bool value);

    /// @brief Read the value of a single-line request.  Returns 0/1 on success, -1 on error.
    int get_value();

    /// @brief Set bit @p index of a multi-line request.
    int set_value(uint32_t index, bool value);

    /// @brief Read bit @p index of a multi-line request.  Returns 0/1 on success, -1 on error.
    int get_value(uint32_t index);

    /**
     * @brief Read/write all line values as a packed bitmask.
     *
     * Bit `i` corresponds to the `i`-th offset given at request time.
     */
    int set_values_mask(uint64_t values);
    int get_values_mask(uint64_t * values);

  private:
    friend class GpioInterface;
    Line(int fd, uint32_t num_lines, std::string log_name);

    int fd_{-1};
    uint32_t num_lines_{0};
    std::string log_name_;
  };

  /**
   * @brief Construct from a full chip path.
   * @param chip_path Device path, e.g. "/dev/gpiochip0".
   * @param eager_open If true, open the chip in the constructor and throw on failure.
   * @throws std::invalid_argument if @p chip_path is empty.
   * @throws std::runtime_error if @p eager_open is true and the chip cannot be opened.
   */
  explicit GpioInterface(const std::string & chip_path, bool eager_open = false);

  /**
   * @brief Construct from a chip number.
   * @param chip_number Index (e.g. 0 → "/dev/gpiochip0").
   */
  explicit GpioInterface(uint8_t chip_number, bool eager_open = false);

  ~GpioInterface();

  GpioInterface(const GpioInterface &) = delete;
  GpioInterface & operator=(const GpioInterface &) = delete;

  /**
   * @brief Request a single line as an output.
   * @param offset      Line number within the chip (e.g. BCM pin number on a Pi).
   * @param initial_value Value driven immediately on request (false=low, true=high).
   * @param drive       Output drive mode (push-pull by default).
   * @return A `Line` handle.  Check `ok()`.
   */
  Line request_output(
    uint32_t offset, bool initial_value = false, GpioDrive drive = GpioDrive::PushPull);

  /**
   * @brief Request a single line as an input.
   * @param offset  Line number within the chip.
   * @param bias    Internal pull bias to apply.
   * @return A `Line` handle.  Check `ok()`.
   */
  Line request_input(uint32_t offset, GpioBias bias = GpioBias::AsIs);

  /**
   * @brief Request multiple lines as outputs in a single bundle (atomic
   *        multi-line `set_values_mask()`).
   */
  Line request_outputs(
    const uint32_t * offsets, uint32_t num_offsets, uint64_t initial_values_mask = 0,
    GpioDrive drive = GpioDrive::PushPull);

  /**
   * @brief Request multiple lines as inputs in a single bundle.
   */
  Line request_inputs(
    const uint32_t * offsets, uint32_t num_offsets, GpioBias bias = GpioBias::AsIs);

  /// @brief Chip path this interface was constructed with (e.g. "/dev/gpiochip0").
  const std::string & chip_path() const { return chip_path_; }

  /**
   * @brief Get the process-wide shared interface for @p chip_path.
   *
   * Returns an existing instance if one is alive, otherwise constructs a new
   * one.  Thread-safe.
   */
  static std::shared_ptr<GpioInterface> get_shared(
    const std::string & chip_path, bool eager_open = false);

  /// @brief Chip-number overload of get_shared().
  static std::shared_ptr<GpioInterface> get_shared(uint8_t chip_number, bool eager_open = false);

private:
  std::string chip_path_;                  ///< Device path (e.g. "/dev/gpiochip0").
  std::string log_name_;                   ///< Logger name (e.g. "linux_gpio_interface.gpiochip0").
  int chip_fd_{-1};                        ///< File descriptor for the open chip.
  std::mutex chip_mutex_;                  ///< Serialises chip open/close and line requests.
  std::atomic<bool> is_connected_{false};  ///< Whether the chip is currently open.
  int last_open_errno_{0};                 ///< Errno from last open failure (for log dedup).

  /// @brief Open the chip fd (mutex must already be held).
  int open_chip_unlocked();

  /// @brief Close the chip fd (mutex must already be held).
  int close_chip_unlocked();

  /// @brief Common request helper.  Returns a `Line` (check ok()).
  Line request_lines(
    const uint32_t * offsets, uint32_t num_offsets, GpioDirection direction, GpioBias bias,
    GpioDrive drive, uint64_t initial_values_mask);

  /// @brief Registry for get_shared(): chip path -> weak handle.
  static std::mutex registry_mutex_;
  static std::map<std::string, std::weak_ptr<GpioInterface>> registry_;
};

}  // namespace linux_gpio_interface
