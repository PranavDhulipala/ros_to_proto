# ROS 2 to Protocol Buffers Converter

A bidirectional conversion system that seamlessly bridges ROS 2 messages and Protocol Buffer messages for cross-platform interoperability.

![ROS2](https://img.shields.io/badge/ROS-Humble-brightgreen)

## Overview

This package provides a framework for converting between ROS 2 messages and Google Protocol Buffers (Protobuf), enabling seamless communication between ROS 2-based systems and other platforms that utilize Protobuf for data serialization. The protos are produced within the ROS ecosystem and can be distributed to external systems facilitating effective cross-platform communication.

## Key Features

- **Bidirectional Conversion**: Convert from ROS 2 messages to Protocol Buffers and vice versa
- **Automatic Type Conversion**: Transparently handles mapping between equivalent ROS 2 and Protobuf message types
- **Dynamic Registration System**: Automatically discovers and registers converters at runtime
- **Comprehensive Type Support**: 
  - Primitive types (int32, float, string, boolean, wstring.)
  - Complex structures (arrays, sequences)
  - Nested messages with recursive conversion
- **Extensible Architecture**: Easily add support for custom message types

## Repository Structure

This repository includes several key components:

- **rosidl_converter_protobuf_py**: Core Python library that implements the conversion between ROS 2 messages and Protobuf. While [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf) provides the C++ typesupport and generates the Protocol Buffer definitions, this package specifically handles the Python-side conversion logic.
- **rosidl_converter_protobuf_py_tests**: Comprehensive test suite for the Python converter.
- **rosidl_typesupport_protobuf_cpp_tests**: Tests for the C++ Protobuf typesupport provided by the external [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf) package
- **data_acquisition_interfaces**: Example ROS 2 interface definitions for data acquisition
- **key_value_interfaces**: Example ROS 2 interface definitions for key-value data structures

## Docker Environment

The included Dockerfile provides a consistent development and testing environment with all dependencies pre-configured.

### What's Included
- Ubuntu 22.04 with ROS 2 Humble built from source
- Protocol Buffer compiler and libraries
- Custom modifications to enable Python Protobuf generation
- Integration with:
  - [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf) - Adds Protobuf typesupport to ROS 2 and generates Protocol Buffer definitions
  - [proto2ros](https://github.com/bdaiinstitute/proto2ros) - Enables conversion from Protobuf to ROS messages
- Eclipse Zenoh for efficient pub/sub communication

### Building the Docker Image
```bash
# From the repository root
docker build -t ros2_proto_converter .
```

### Running the Container
```bash
# Run with interactive shell
docker run -it ros2_proto_converter
```

## Roadmap

The following features are planned in the future:

- Support for ROS 2 service interface conversion
- Support for ROS 2 action interface conversion
- Direct serialization/deserialization utilities

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Acknowledgments

- The ROS 2 community for their excellent messaging system
- Google for Protocol Buffers
- Contributors to rosidl_typesupport_protobuf, proto2ros, rosidl_generator_mypy
- The rosidl_generator_mypy project for its implementation structure that inspired this work