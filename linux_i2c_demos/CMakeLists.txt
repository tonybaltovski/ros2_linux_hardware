cmake_minimum_required(VERSION 3.14)
project(linux_i2c_demos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(linux_i2c_devices REQUIRED)
find_package(linux_i2c_interface REQUIRED)

include_directories(include)

include_directories(${linux_i2c_devices_INCLUDE_DIRS})
include_directories(${linux_i2c_interface_INCLUDE_DIRS})

add_executable(screen_rosout_logger src/screen_rosout_logger.cpp)
ament_target_dependencies(screen_rosout_logger linux_i2c_devices linux_i2c_interface rcl_interfaces rclcpp std_msgs)

add_executable(screen_from_str_topic src/screen_from_str_topic.cpp)
ament_target_dependencies(screen_from_str_topic linux_i2c_devices linux_i2c_interface rclcpp std_msgs)

add_executable(pca9685_servo_control src/pca9685_servo_control.cpp)
ament_target_dependencies(pca9685_servo_control linux_i2c_devices linux_i2c_interface rclcpp std_msgs)

install(TARGETS
  screen_rosout_logger screen_from_str_topic pca9685_servo_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
