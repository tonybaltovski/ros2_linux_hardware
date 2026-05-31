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

#include "linux_iio_devices/iio_device.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace linux_iio_devices
{

namespace
{
constexpr const char * IIO_SYSFS_ROOT = "/sys/bus/iio/devices";

std::string trim(std::string s)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}
}  // namespace

std::optional<std::string> resolve_by_name(const std::string & name)
{
  DIR * dir = ::opendir(IIO_SYSFS_ROOT);
  if (!dir)
  {
    return std::nullopt;
  }
  std::optional<std::string> result;
  while (auto * entry = ::readdir(dir))
  {
    const std::string entry_name = entry->d_name;
    if (entry_name.rfind("iio:device", 0) != 0)
    {
      continue;
    }
    const std::string device_path = std::string(IIO_SYSFS_ROOT) + "/" + entry_name;
    auto reported = read_string(device_path + "/name");
    if (reported && *reported == name)
    {
      result = device_path;
      break;
    }
  }
  ::closedir(dir);
  return result;
}

std::optional<std::string> read_string(const std::string & path)
{
  std::ifstream in(path);
  if (!in)
  {
    return std::nullopt;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return trim(ss.str());
}

std::optional<double> read_double(const std::string & path)
{
  auto s = read_string(path);
  if (!s)
  {
    return std::nullopt;
  }
  try
  {
    return std::stod(*s);
  }
  catch (const std::exception &)
  {
    return std::nullopt;
  }
}

std::optional<int64_t> read_int(const std::string & path)
{
  auto s = read_string(path);
  if (!s)
  {
    return std::nullopt;
  }
  try
  {
    return static_cast<int64_t>(std::stoll(*s));
  }
  catch (const std::exception &)
  {
    return std::nullopt;
  }
}

bool write_string(const std::string & path, const std::string & value)
{
  std::ofstream out(path);
  if (!out)
  {
    return false;
  }
  out << value;
  return out.good();
}

BufferedReader::BufferedReader(const std::string & device_path, std::size_t record_size_bytes)
: record_size_(record_size_bytes)
{
  // device_path is /sys/bus/iio/devices/iio:deviceN; the matching char device
  // is /dev/iio:deviceN, with the same suffix.
  const auto pos = device_path.rfind('/');
  if (pos == std::string::npos)
  {
    throw std::runtime_error("BufferedReader: malformed device path: " + device_path);
  }
  const std::string dev_path = "/dev/" + device_path.substr(pos + 1);
  fd_ = ::open(dev_path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0)
  {
    throw std::runtime_error(
      "BufferedReader: failed to open " + dev_path + ": " + ::strerror(errno));
  }
}

BufferedReader::~BufferedReader()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
  }
}

bool BufferedReader::wait_record(std::vector<uint8_t> & out, int timeout_ms)
{
  ::pollfd pfd{fd_, POLLIN, 0};
  const int rc = ::poll(&pfd, 1, timeout_ms);
  if (rc <= 0)
  {
    return false;
  }
  out.resize(record_size_);
  ssize_t total = 0;
  while (total < static_cast<ssize_t>(record_size_))
  {
    const ssize_t n = ::read(fd_, out.data() + total, record_size_ - total);
    if (n <= 0)
    {
      // EAGAIN here means the kernel handed us less than a full record; treat
      // as a short read and let the caller try again.
      return false;
    }
    total += n;
  }
  return true;
}

}  // namespace linux_iio_devices
