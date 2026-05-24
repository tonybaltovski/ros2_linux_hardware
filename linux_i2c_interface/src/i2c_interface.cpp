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

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "linux_i2c_interface/i2c_interface.hpp"
#include "rclcpp/logging.hpp"

namespace linux_i2c_interface
{
namespace
{

/// @brief Strip the directory portion of an I2C bus path ("/dev/i2c-1" -> "i2c-1").
std::string bus_basename(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

/// @brief Errno values that indicate the bus/fd is in a bad state and should be reopened.
bool is_bus_fault(int err)
{
  return err == EIO || err == ENXIO || err == ENODEV || err == EBADF || err == ETIMEDOUT ||
         err == EREMOTEIO;
}

/// @brief Retry a syscall-style call as long as it fails with EINTR.
template <typename Fn>
auto retry_eintr(Fn && fn) -> decltype(fn())
{
  decltype(fn()) ret;
  do
  {
    ret = fn();
  } while (ret < 0 && errno == EINTR);
  return ret;
}

}  // namespace

// ---------------------------------------------------------------------------
// Transaction
// ---------------------------------------------------------------------------

I2cInterface::Transaction::Transaction(I2cInterface & iface, uint8_t device_id)
: iface_(&iface), lock_(iface.i2c_mutex_), ok_(false)
{
  if (iface_->open_bus_unlocked() < 0)
  {
    return;
  }
  ok_ = (iface_->ensure_device_unlocked(device_id) == 0);
}

I2cInterface::Transaction::Transaction(Transaction && other) noexcept
: iface_(other.iface_), lock_(std::move(other.lock_)), ok_(other.ok_)
{
  other.iface_ = nullptr;
  other.ok_ = false;
}

I2cInterface::Transaction & I2cInterface::Transaction::operator=(Transaction && other) noexcept
{
  if (this != &other)
  {
    iface_ = other.iface_;
    lock_ = std::move(other.lock_);
    ok_ = other.ok_;
    other.iface_ = nullptr;
    other.ok_ = false;
  }
  return *this;
}

I2cInterface::Transaction::~Transaction() = default;

int I2cInterface::Transaction::write(uint8_t address, const void * data, uint32_t count)
{
  if (!ok_)
  {
    errno = ENOTCONN;
    return -1;
  }
  return iface_->write_to_bus_unlocked(address, data, count);
}

int I2cInterface::Transaction::write_cmd(uint8_t address) { return write(address, nullptr, 0); }

int I2cInterface::Transaction::read(uint8_t address, void * data, uint32_t count)
{
  if (!ok_)
  {
    errno = ENOTCONN;
    return -1;
  }
  return iface_->read_from_bus_unlocked(address, data, count);
}

int I2cInterface::Transaction::write_raw(const void * data, uint32_t count)
{
  if (!ok_)
  {
    errno = ENOTCONN;
    return -1;
  }
  return iface_->write_raw_unlocked(data, count);
}

// ---------------------------------------------------------------------------
// I2cInterface
// ---------------------------------------------------------------------------

I2cInterface::I2cInterface(const std::string & i2c_bus, bool eager_open)
: i2c_bus_(i2c_bus), log_name_("linux_i2c_interface." + bus_basename(i2c_bus_))
{
  if (i2c_bus_.empty())
  {
    throw std::invalid_argument("I2cInterface: bus path must not be empty");
  }
  if (eager_open)
  {
    const std::lock_guard<std::mutex> lock(i2c_mutex_);
    if (open_bus_unlocked() < 0)
    {
      const int err = errno;
      throw std::runtime_error("I2cInterface: failed to open " + i2c_bus_ + ": " + strerror(err));
    }
  }
}

I2cInterface::I2cInterface(uint8_t i2c_bus_number, bool eager_open)
: I2cInterface("/dev/i2c-" + std::to_string(i2c_bus_number), eager_open)
{
}

std::mutex I2cInterface::registry_mutex_;
std::map<std::string, std::weak_ptr<I2cInterface>> I2cInterface::registry_;

std::shared_ptr<I2cInterface> I2cInterface::get_shared(const std::string & i2c_bus, bool eager_open)
{
  const std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = registry_.find(i2c_bus);
  if (it != registry_.end())
  {
    if (auto existing = it->second.lock())
    {
      return existing;
    }
  }
  auto created = std::make_shared<I2cInterface>(i2c_bus, eager_open);
  registry_[i2c_bus] = created;
  return created;
}

std::shared_ptr<I2cInterface> I2cInterface::get_shared(uint8_t i2c_bus_number, bool eager_open)
{
  return get_shared("/dev/i2c-" + std::to_string(i2c_bus_number), eager_open);
}

I2cInterface::~I2cInterface()
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (is_connected_)
  {
    close_bus_unlocked();
  }
}

int I2cInterface::close_bus_unlocked()
{
  int ret = close(i2c_fd_);
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(rclcpp::get_logger(log_name_), "%s: Failed to close: %s", __func__, strerror(err));
    errno = err;
  }
  // On Linux, close() releases the fd even on error — mark disconnected unconditionally.
  is_connected_ = false;
  current_device_id_ = -1;
  i2c_fd_ = -1;
  return ret;
}

int I2cInterface::open_bus_unlocked()
{
  if (is_connected_)
  {
    return 0;
  }

  i2c_fd_ = retry_eintr([&] { return open(i2c_bus_.c_str(), O_RDWR | O_CLOEXEC); });

  if (i2c_fd_ < 0)
  {
    const int err = errno;
    // Dedup repeated identical open failures (e.g. missing bus in a hot loop).
    if (err != last_open_errno_)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(log_name_), "%s: Could not open: %s with error: %s", __func__,
        i2c_bus_.c_str(), strerror(err));
      last_open_errno_ = err;
    }
    errno = err;
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger(log_name_), "%s: Connected to %s", __func__, i2c_bus_.c_str());
  is_connected_ = true;
  current_device_id_ = -1;
  last_open_errno_ = 0;
  return 0;
}

I2cInterface::Transaction I2cInterface::begin_transaction(uint8_t device_id)
{
  return Transaction(*this, device_id);
}

int I2cInterface::set_device_id_unlocked(const uint8_t device_id)
{
  int ret = retry_eintr([&] { return ioctl(i2c_fd_, I2C_SLAVE, device_id); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to set device 0x%02x: %s", __func__,
      static_cast<int>(device_id), strerror(err));
    current_device_id_ = -1;
    if (is_bus_fault(err))
    {
      close_bus_unlocked();
    }
    errno = err;
    return -1;
  }
  current_device_id_ = device_id;
  return 0;
}

int I2cInterface::ensure_device_unlocked(const uint8_t device_id)
{
  if (current_device_id_ == static_cast<int>(device_id))
  {
    return 0;
  }
  return set_device_id_unlocked(device_id);
}

int I2cInterface::read_from_bus_unlocked(const uint8_t address, void * data, uint32_t count)
{
  if (current_device_id_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(log_name_), "%s: No slave selected", __func__);
    errno = EINVAL;
    return -1;
  }

  // Use a single I2C_RDWR ioctl so the register-pointer write and the read are
  // joined with a repeated-START (required by many sensors).
  uint8_t reg = address;
  struct i2c_msg msgs[2];
  msgs[0].addr = static_cast<uint16_t>(current_device_id_);
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &reg;
  msgs[1].addr = static_cast<uint16_t>(current_device_id_);
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<uint16_t>(count);
  msgs[1].buf = static_cast<uint8_t *>(data);

  struct i2c_rdwr_ioctl_data set;
  set.msgs = msgs;
  set.nmsgs = 2;

  int ret = retry_eintr([&] { return ioctl(i2c_fd_, I2C_RDWR, &set); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Combined read failed: %s", __func__, strerror(err));
    if (is_bus_fault(err))
    {
      close_bus_unlocked();
    }
    errno = err;
    return -1;
  }
  return 0;
}

int I2cInterface::write_to_bus(const uint8_t device_id, const uint8_t address)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (open_bus_unlocked() < 0)
  {
    return -1;
  }
  int ret = ensure_device_unlocked(device_id);
  if (ret < 0)
  {
    return ret;
  }
  return write_to_bus_unlocked(address, nullptr, 0);
}

int I2cInterface::write_to_bus_unlocked(const uint8_t address, const void * data, uint32_t count)
{
  // Issue [addr, data...] as a SINGLE write() = one I2C transaction. Splitting
  // into two write()s makes the slave interpret data[0] as a new register pointer.
  std::vector<uint8_t> buf(1 + count);
  buf[0] = address;
  if (count > 0)
  {
    std::memcpy(buf.data() + 1, data, count);
  }

  int ret = retry_eintr([&] { return write(i2c_fd_, buf.data(), buf.size()); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to write to device: %s", __func__, strerror(err));
    if (is_bus_fault(err))
    {
      close_bus_unlocked();
    }
    errno = err;
    return -1;
  }
  if (static_cast<size_t>(ret) != buf.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Short write to device, expected %zu, got %d", __func__,
      buf.size(), ret);
    errno = EIO;
    return -1;
  }
  return 0;
}

int I2cInterface::write_raw_unlocked(const void * data, uint32_t count)
{
  int ret = retry_eintr([&] { return write(i2c_fd_, data, count); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to write to device: %s", __func__, strerror(err));
    if (is_bus_fault(err))
    {
      close_bus_unlocked();
    }
    errno = err;
    return -1;
  }
  else if (static_cast<int>(count) != ret)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Short write to device, expected %u, got %d", __func__,
      count, ret);
    errno = EIO;
    return -1;
  }
  return 0;
}

}  // namespace linux_i2c_interface
