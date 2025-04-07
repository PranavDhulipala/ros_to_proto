cmake_minimum_required(VERSION 3.8)
project(rosidl_typesupport_protobuf_cpp_tests)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Add Unicode support flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-fexec-charset=UTF-8)
  add_compile_options(-finput-charset=UTF-8)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fexec-charset=UTF-8)
  add_compile_options(-finput-charset=UTF-8)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(data_acquisition_interfaces REQUIRED)
find_package(key_value_interfaces REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# Add the test executable
ament_add_gtest(test_data_acquisition_conversion
  test/test_data_acquisition_conversion.cpp
)

# Link dependencies
ament_target_dependencies(test_data_acquisition_conversion
  rclcpp
  data_acquisition_interfaces
  key_value_interfaces
  rosidl_runtime_cpp
)

ament_package()