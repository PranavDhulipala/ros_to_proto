cmake_minimum_required(VERSION 3.8)
project(data_acquisition_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_converter_protobuf_py REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(key_value_interfaces REQUIRED)

# Gather the message definitions
file(GLOB_RECURSE INTERFACE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES key_value_interfaces
)

ament_package()

