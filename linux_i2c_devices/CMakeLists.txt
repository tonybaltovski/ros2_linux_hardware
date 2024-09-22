cmake_minimum_required(VERSION 3.5)
project(linux_i2c_devices)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(rcl_interfaces REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(linux_i2c_interface REQUIRED)


include_directories(include)

include_directories(${linux_i2c_interface_INCLUDE_DIRS})


add_library(${PROJECT_NAME} SHARED
  src/lcm1602.cpp
)

set(DEPENDENCIES
  linux_i2c_interface
)



#ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()