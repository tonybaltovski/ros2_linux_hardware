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
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "linux_gpio_interface/gpio_interface.hpp"
#include "rclcpp/logging.hpp"

namespace linux_gpio_interface
{
namespace
{

constexpr const char * kConsumer = "linux_gpio_interface";

std::string chip_basename(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

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

uint64_t direction_flags(GpioDirection dir)
{
  return dir == GpioDirection::Output ? GPIO_V2_LINE_FLAG_OUTPUT : GPIO_V2_LINE_FLAG_INPUT;
}

uint64_t bias_flags(GpioBias bias)
{
  switch (bias)
  {
    case GpioBias::Disabled:
      return GPIO_V2_LINE_FLAG_BIAS_DISABLED;
    case GpioBias::PullUp:
      return GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
    case GpioBias::PullDown:
      return GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
    case GpioBias::AsIs:
    default:
      return 0;
  }
}

uint64_t drive_flags(GpioDrive drive)
{
  switch (drive)
  {
    case GpioDrive::OpenDrain:
      return GPIO_V2_LINE_FLAG_OPEN_DRAIN;
    case GpioDrive::OpenSource:
      return GPIO_V2_LINE_FLAG_OPEN_SOURCE;
    case GpioDrive::PushPull:
    default:
      return 0;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Line
// ---------------------------------------------------------------------------

GpioInterface::Line::Line(int fd, uint32_t num_lines, std::string log_name)
: fd_(fd), num_lines_(num_lines), log_name_(std::move(log_name))
{
}

GpioInterface::Line::Line(Line && other) noexcept
: fd_(other.fd_), num_lines_(other.num_lines_), log_name_(std::move(other.log_name_))
{
  other.fd_ = -1;
  other.num_lines_ = 0;
}

GpioInterface::Line & GpioInterface::Line::operator=(Line && other) noexcept
{
  if (this != &other)
  {
    if (fd_ >= 0)
    {
      close(fd_);
    }
    fd_ = other.fd_;
    num_lines_ = other.num_lines_;
    log_name_ = std::move(other.log_name_);
    other.fd_ = -1;
    other.num_lines_ = 0;
  }
  return *this;
}

GpioInterface::Line::~Line()
{
  if (fd_ >= 0)
  {
    close(fd_);
  }
}

int GpioInterface::Line::set_values_mask(uint64_t values)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  struct gpio_v2_line_values vals;
  std::memset(&vals, 0, sizeof(vals));
  vals.bits = values;
  vals.mask = (num_lines_ >= 64) ? ~0ULL : ((1ULL << num_lines_) - 1ULL);

  int ret = retry_eintr([&] { return ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to set GPIO values: %s", __func__, strerror(err));
    errno = err;
    return -1;
  }
  return 0;
}

int GpioInterface::Line::get_values_mask(uint64_t * values)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  if (values == nullptr)
  {
    errno = EINVAL;
    return -1;
  }
  struct gpio_v2_line_values vals;
  std::memset(&vals, 0, sizeof(vals));
  vals.mask = (num_lines_ >= 64) ? ~0ULL : ((1ULL << num_lines_) - 1ULL);

  int ret = retry_eintr([&] { return ioctl(fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to get GPIO values: %s", __func__, strerror(err));
    errno = err;
    return -1;
  }
  *values = vals.bits & vals.mask;
  return 0;
}

int GpioInterface::Line::set_value(bool value)
{
  return set_values_mask(value ? 1ULL : 0ULL);
}

int GpioInterface::Line::get_value()
{
  uint64_t v = 0;
  if (get_values_mask(&v) < 0)
  {
    return -1;
  }
  return static_cast<int>(v & 1ULL);
}

int GpioInterface::Line::set_value(uint32_t index, bool value)
{
  if (index >= num_lines_)
  {
    errno = EINVAL;
    return -1;
  }
  // Read-modify-write so we don't disturb other lines in the bundle.
  uint64_t current = 0;
  if (get_values_mask(&current) < 0)
  {
    return -1;
  }
  const uint64_t bit = 1ULL << index;
  const uint64_t next = value ? (current | bit) : (current & ~bit);
  return set_values_mask(next);
}

int GpioInterface::Line::get_value(uint32_t index)
{
  if (index >= num_lines_)
  {
    errno = EINVAL;
    return -1;
  }
  uint64_t v = 0;
  if (get_values_mask(&v) < 0)
  {
    return -1;
  }
  return static_cast<int>((v >> index) & 1ULL);
}

// ---------------------------------------------------------------------------
// GpioInterface
// ---------------------------------------------------------------------------

GpioInterface::GpioInterface(const std::string & chip_path, bool eager_open)
: chip_path_(chip_path), log_name_("linux_gpio_interface." + chip_basename(chip_path_))
{
  if (chip_path_.empty())
  {
    throw std::invalid_argument("GpioInterface: chip path must not be empty");
  }
  if (eager_open)
  {
    const std::lock_guard<std::mutex> lock(chip_mutex_);
    if (open_chip_unlocked() < 0)
    {
      const int err = errno;
      throw std::runtime_error(
        "GpioInterface: failed to open " + chip_path_ + ": " + strerror(err));
    }
  }
}

GpioInterface::GpioInterface(uint8_t chip_number, bool eager_open)
: GpioInterface("/dev/gpiochip" + std::to_string(chip_number), eager_open)
{
}

GpioInterface::~GpioInterface()
{
  const std::lock_guard<std::mutex> lock(chip_mutex_);
  if (is_connected_)
  {
    close_chip_unlocked();
  }
}

std::mutex GpioInterface::registry_mutex_;
std::map<std::string, std::weak_ptr<GpioInterface>> GpioInterface::registry_;

std::shared_ptr<GpioInterface> GpioInterface::get_shared(
  const std::string & chip_path, bool eager_open)
{
  const std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = registry_.find(chip_path);
  if (it != registry_.end())
  {
    if (auto existing = it->second.lock())
    {
      return existing;
    }
  }
  auto created = std::make_shared<GpioInterface>(chip_path, eager_open);
  registry_[chip_path] = created;
  return created;
}

std::shared_ptr<GpioInterface> GpioInterface::get_shared(uint8_t chip_number, bool eager_open)
{
  return get_shared("/dev/gpiochip" + std::to_string(chip_number), eager_open);
}

int GpioInterface::open_chip_unlocked()
{
  if (is_connected_)
  {
    return 0;
  }
  chip_fd_ = retry_eintr([&] { return open(chip_path_.c_str(), O_RDWR | O_CLOEXEC); });
  if (chip_fd_ < 0)
  {
    const int err = errno;
    if (err != last_open_errno_)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(log_name_), "%s: Could not open: %s with error: %s", __func__,
        chip_path_.c_str(), strerror(err));
      last_open_errno_ = err;
    }
    errno = err;
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger(log_name_), "%s: Connected to %s", __func__, chip_path_.c_str());
  is_connected_ = true;
  last_open_errno_ = 0;
  return 0;
}

int GpioInterface::close_chip_unlocked()
{
  int ret = close(chip_fd_);
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(rclcpp::get_logger(log_name_), "%s: Failed to close: %s", __func__, strerror(err));
    errno = err;
  }
  is_connected_ = false;
  chip_fd_ = -1;
  return ret;
}

GpioInterface::Line GpioInterface::request_lines(
  const uint32_t * offsets, uint32_t num_offsets, GpioDirection direction, GpioBias bias,
  GpioDrive drive, uint64_t initial_values_mask)
{
  if (offsets == nullptr || num_offsets == 0 || num_offsets > GPIO_V2_LINES_MAX)
  {
    errno = EINVAL;
    return Line();
  }

  const std::lock_guard<std::mutex> lock(chip_mutex_);
  if (open_chip_unlocked() < 0)
  {
    return Line();
  }

  struct gpio_v2_line_request req;
  std::memset(&req, 0, sizeof(req));
  for (uint32_t i = 0; i < num_offsets; ++i)
  {
    req.offsets[i] = offsets[i];
  }
  req.num_lines = num_offsets;
  std::strncpy(req.consumer, kConsumer, sizeof(req.consumer) - 1);

  req.config.flags = direction_flags(direction) | bias_flags(bias);
  if (direction == GpioDirection::Output)
  {
    req.config.flags |= drive_flags(drive);
    // Attach initial output values so we don't glitch when the line switches
    // to output mode.
    req.config.num_attrs = 1;
    req.config.attrs[0].mask =
      (num_offsets >= 64) ? ~0ULL : ((1ULL << num_offsets) - 1ULL);
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = initial_values_mask;
  }

  int ret = retry_eintr([&] { return ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req); });
  if (ret < 0)
  {
    const int err = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Failed to request %u line(s) (first offset %u): %s",
      __func__, num_offsets, offsets[0], strerror(err));
    errno = err;
    return Line();
  }
  if (req.fd < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(log_name_), "%s: Kernel returned invalid line fd", __func__);
    errno = EBADF;
    return Line();
  }

  return Line(req.fd, num_offsets, log_name_);
}

GpioInterface::Line GpioInterface::request_output(
  uint32_t offset, bool initial_value, GpioDrive drive)
{
  return request_lines(
    &offset, 1, GpioDirection::Output, GpioBias::AsIs, drive, initial_value ? 1ULL : 0ULL);
}

GpioInterface::Line GpioInterface::request_input(uint32_t offset, GpioBias bias)
{
  return request_lines(&offset, 1, GpioDirection::Input, bias, GpioDrive::PushPull, 0ULL);
}

GpioInterface::Line GpioInterface::request_outputs(
  const uint32_t * offsets, uint32_t num_offsets, uint64_t initial_values_mask, GpioDrive drive)
{
  return request_lines(
    offsets, num_offsets, GpioDirection::Output, GpioBias::AsIs, drive, initial_values_mask);
}

GpioInterface::Line GpioInterface::request_inputs(
  const uint32_t * offsets, uint32_t num_offsets, GpioBias bias)
{
  return request_lines(offsets, num_offsets, GpioDirection::Input, bias, GpioDrive::PushPull, 0ULL);
}

}  // namespace linux_gpio_interface
