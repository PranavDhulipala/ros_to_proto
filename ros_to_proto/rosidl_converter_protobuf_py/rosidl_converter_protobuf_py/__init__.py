"""Python package for rosidl_converter_protobuf_py."""

import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Import the generate_py function so it can be used from rosidl_converter_protobuf_py.generate_py
from rosidl_converter_protobuf_py.generate import generate_py

# Import the conversions module to make it available
from rosidl_converter_protobuf_py.conversions import (
    register_converter,
    convert,
    register_primitive_conversion,
    get_converter,
    get_proto_type_for_ros,
    get_ros_type_for_proto,
    list_converters
)

__all__ = [
    "generate_py",
    "register_converter",
    "convert",
    "register_primitive_conversion",
    "get_converter",
    "get_proto_type_for_ros",
    "get_ros_type_for_proto",
    "list_converters"
]