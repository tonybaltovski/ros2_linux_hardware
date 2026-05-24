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

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace linux_i2c_interface
{

/**
 * @class I2cInterface
 * @brief Provides thread-safe read/write access to a Linux I2C bus.
 *
 * The underlying `/dev/i2c-N` file descriptor is owned by the interface:
 * it is opened lazily on the first transaction and closed automatically
 * when the `I2cInterface` is destroyed.  If a transient bus error causes
 * the fd to be closed mid-run, the next transaction transparently reopens
 * it.  Drivers never call open/close themselves.
 *
 * Two access patterns are supported:
 *
 *  1. **Single-call atomic operation** (`write_to_bus(device_id, addr)`):
 *     atomically opens (if needed), selects the slave, and performs the
 *     transfer under the internal mutex.  Use this for one-shot single-byte
 *     writes to a known device.
 *
 *  2. **Transactions** (preferred for multi-step sequences and for sharing
 *     one interface across many devices): `begin_transaction(device_id)`
 *     returns an RAII guard that holds the bus mutex for its lifetime and
 *     keeps the requested slave selected.  All `Transaction` methods are
 *     unlocked primitives, so multi-step sequences (e.g. SSD1306
 *     column+page+data) execute as one logical bus transaction.
 *
 * **Error reporting.** All transfer methods return 0 on success and -1 on
 * failure.  On failure, `errno` always carries a cause:
 *   - syscall errnos (`EIO`, `ENXIO`, `ENODEV`, `EBADF`, `ETIMEDOUT`, `ENOENT`,
 *     `EACCES`, ...) propagate unchanged from `open()` / `ioctl()` / `read()` /
 *     `write()`;
 *   - `ENOTCONN` means the transaction failed to acquire the bus or select the
 *     slave (use a fresh `begin_transaction()`);
 *   - `EIO` is also used for short writes;
 *   - `EINVAL` means a programming error (e.g. read with no slave selected).
 *
 * **Sharing one bus across many devices.** All drivers on the same physical
 * bus must share the *same* `I2cInterface` instance, otherwise each gets its
 * own mutex and transfers will interleave on the wire.  Use `get_shared()` to
 * obtain a process-wide `std::shared_ptr<I2cInterface>` keyed by bus path; it
 * returns the same instance for repeated calls and frees it when the last
 * driver releases its `shared_ptr`.
 */
class I2cInterface
{
public:
  /**
   * @class Transaction
   * @brief RAII scoped bus reservation for safe multi-step I2C sequences.
   *
   * Construction acquires the bus mutex and selects the target slave; the lock
   * is released on destruction (or move).  Transfer methods are unlocked
   * primitives run while the lock is held.  Move-only; hold only as long as
   * needed since other drivers cannot use the same bus meanwhile.
   */
  class Transaction
  {
  public:
    Transaction(const Transaction &) = delete;
    Transaction & operator=(const Transaction &) = delete;
    Transaction(Transaction && other) noexcept;
    Transaction & operator=(Transaction && other) noexcept;
    ~Transaction();

    /// @brief True if the bus is open and the slave was selected successfully.
    bool ok() const { return ok_; }

    /// @brief Write a register address followed by a payload.
    int write(uint8_t address, const void * data, uint32_t count);

    /// @brief Write a single command byte (no payload).
    int write_cmd(uint8_t address);

    /// @brief Read bytes from the selected device.
    int read(uint8_t address, void * data, uint32_t count);

    /// @brief Write a raw buffer as a single I2C transfer (no register prefix).
    int write_raw(const void * data, uint32_t count);

  private:
    friend class I2cInterface;
    Transaction(I2cInterface & iface, uint8_t device_id);

    I2cInterface * iface_;
    std::unique_lock<std::mutex> lock_;
    bool ok_;
  };

  /**
   * @brief Construct an I2C interface from a full device path.
   * @param i2c_bus Device path, e.g. "/dev/i2c-1".
   * @param eager_open If true, open the bus in the constructor and throw on
   *        failure; if false (default), the bus is opened lazily on the first
   *        transaction.  Use eager open to fail fast on a typo or permissions
   *        problem at startup instead of silently failing every transfer.
   * @throws std::invalid_argument if @p i2c_bus is empty.
   * @throws std::runtime_error if @p eager_open is true and the bus cannot be opened.
   */
  explicit I2cInterface(const std::string & i2c_bus, bool eager_open = false);

  /**
   * @brief Construct an I2C interface from a bus number.
   * @param i2c_bus_number Bus index (e.g. 1 → "/dev/i2c-1").
   * @param eager_open See string-path constructor.
   * @throws std::runtime_error if @p eager_open is true and the bus cannot be opened.
   */
  explicit I2cInterface(uint8_t i2c_bus_number, bool eager_open = false);

  /// @brief Close the bus fd if it is open.
  ~I2cInterface();

  I2cInterface(const I2cInterface &) = delete;
  I2cInterface & operator=(const I2cInterface &) = delete;

  /**
   * @brief Begin a scoped multi-step I2C transaction.
   *
   * Lazily opens the bus if needed, acquires the bus mutex, and selects
   * @p device_id.  All subsequent calls via the returned guard execute
   * atomically; the lock is released when the guard is destroyed.
   *
   * @return RAII transaction guard; check `ok()` before use.
   */
  Transaction begin_transaction(uint8_t device_id);

  /**
   * @brief Atomically open (if needed), select @p device_id, and write a single
   *        command byte.  For multi-byte sequences use begin_transaction().
   */
  int write_to_bus(uint8_t device_id, uint8_t address);

  /// @brief Device path this interface was constructed with (e.g. "/dev/i2c-1").
  const std::string & bus_name() const { return i2c_bus_; }

  /**
   * @brief Get the process-wide shared interface for @p i2c_bus.
   *
   * Returns the existing instance if one is still alive, otherwise constructs
   * a new one.  Ensures every driver on the same physical bus shares one
   * mutex.  Thread-safe.
   *
   * @throws std::invalid_argument / std::runtime_error per the constructor.
   */
  static std::shared_ptr<I2cInterface> get_shared(
    const std::string & i2c_bus, bool eager_open = false);

  /// @brief Bus-number overload of get_shared().
  static std::shared_ptr<I2cInterface> get_shared(uint8_t i2c_bus_number, bool eager_open = false);

private:
  std::string i2c_bus_;                    ///< Device path (e.g. "/dev/i2c-1").
  std::string log_name_;                   ///< Logger name (e.g. "linux_i2c_interface.i2c-1").
  int i2c_fd_{-1};                         ///< File descriptor for the open bus.
  std::mutex i2c_mutex_;                   ///< Serialises all bus operations.
  std::atomic<bool> is_connected_{false};  ///< Whether the bus is currently open.
  int current_device_id_{-1};              ///< Last selected slave (cache, -1 = unknown).
  int last_open_errno_{0};                 ///< Errno from last open failure (for log dedup).

  /// @brief Open the bus fd (mutex must already be held). No-op if open.
  int open_bus_unlocked();

  /// @brief Close the bus (mutex must already be held).
  int close_bus_unlocked();

  /// @brief Select the active slave (mutex must already be held).
  int set_device_id_unlocked(uint8_t device_id);

  /// @brief Select the active slave only if it differs from the cached one.
  int ensure_device_unlocked(uint8_t device_id);

  /// @brief Address + payload write (mutex must already be held).
  int write_to_bus_unlocked(uint8_t address, const void * data, uint32_t count);

  /// @brief Register read (mutex must already be held).
  int read_from_bus_unlocked(uint8_t address, void * data, uint32_t count);

  /// @brief Raw write (mutex must already be held).
  int write_raw_unlocked(const void * data, uint32_t count);

  /// @brief Registry for get_shared(): bus path -> weak handle.
  static std::mutex registry_mutex_;
  static std::map<std::string, std::weak_ptr<I2cInterface>> registry_;
};

}  // namespace linux_i2c_interface
