# Register the protobuf python converter with the ROS IDL generator
macro(rosidl_converter_protobuf_py_extras BIN GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)

  ament_register_extension(
    "rosidl_generate_idl_interfaces"
    "rosidl_converter_protobuf_py"
    "rosidl_converter_protobuf_py_generate_interfaces.cmake")
  
  normalize_path(BIN "${BIN}")
  set(rosidl_converter_protobuf_py_BIN "${BIN}")
  
  set(rosidl_converter_protobuf_py_GENERATOR_FILES "")
  foreach(_generator_file ${GENERATOR_FILES})
    normalize_path(_generator_file "${_generator_file}")
    list(APPEND rosidl_converter_protobuf_py_GENERATOR_FILES "${_generator_file}")
  endforeach()
  
  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_converter_protobuf_py_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()