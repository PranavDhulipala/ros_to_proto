cmake_minimum_required(VERSION 3.5)
project(rosidl_converter_protobuf_py)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_py REQUIRED)

# Handle Protobuf static disable option
if(DEFINED ENV{PROTOBUF_STATIC_DISABLE})
  set(PROTOBUF_STATIC_DISABLE $ENV{PROTOBUF_STATIC_DISABLE}
    CACHE BOOL "If Protobuf Static should be disabled.")
else()
  set(PROTOBUF_STATIC_DISABLE FALSE
    CACHE BOOL "If Protobuf Static should be disabled.")
endif()

if(PROTOBUF_STATIC_DISABLE)
  ament_package()
  message(STATUS "Protobuf static typesupport implementation explicitly disabled - skipping '${PROJECT_NAME}'")
  return()
endif()

# Export dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_py)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Register with ament index as a generator package
ament_index_register_resource("rosidl_generator_packages")

# Configure the extras file
configure_file(
  "rosidl_converter_protobuf_py-extras.cmake.in"
  "rosidl_converter_protobuf_py-extras.cmake"
  @ONLY
)

# Install CMake files DIRECTLY to share/rosidl_converter_protobuf_py/cmake
install(
  FILES
    "cmake/rosidl_converter_protobuf_py_generate_interfaces.cmake"
    "cmake/register_protobuf_py.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_converter_protobuf_py-extras.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Install resource files
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install the command-line tool
install(
  PROGRAMS bin/rosidl_converter_protobuf_py
  DESTINATION lib/rosidl_converter_protobuf_py
)

ament_package(
  CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/rosidl_converter_protobuf_py-extras.cmake"
)