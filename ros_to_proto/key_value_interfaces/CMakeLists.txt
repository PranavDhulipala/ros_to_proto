cmake_minimum_required(VERSION 3.8)
project(key_value_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_converter_protobuf_py REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gather the message definitions
file(GLOB_RECURSE INTERFACE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()

