"""Generate Python type support for Protocol Buffers in ROS 2.
This module provides functions to generate Python files for converting
ROS 2 messages to Protocol Buffers and vice versa. It also handles
the generation of __init__.py files for package structure.
"""
from io import StringIO
import os
import re
import json
import errno
import logging
import traceback
from em import Interpreter

logger = logging.getLogger("rosidl_converter_protobuf_py.generator")

def ros_type_from_namespaced_type_py(namespaced_type):
    """
    Convert a NamespacedType to a Python ROS message type string.

    Example:
        If namespaced_type.namespaces = ['data_acquisition_interfaces', 'msg']
        and namespaced_type.name = 'DataAcquisitionData',
        the returned value will be:
            'data_acquisition_interfaces.msg.DataAcquisitionData'
    """
    return f"{'.'.join(namespaced_type.namespaces)}.{namespaced_type.name}"

def protobuf_type_from_namespaced_type_py(namespaced_type):
    """
    Convert a NamespacedType to the Python module and class name for a Protobuf message.

    This function assumes that the generated protobuf module for the message is named by
    appending '_pb2' to the message name.

    Example:
        If namespaced_type.namespaces = ['data_acquisition_interfaces', 'msg']
        and namespaced_type.name = 'DataAcquisitionData',
        the returned tuple will be:
            ('data_acquisition_interfaces.msg.DataAcquisitionData_pb2', 'DataAcquisitionData')
    """
    module_path = f"{'.'.join(namespaced_type.namespaces)}.{namespaced_type.name}_pb2"
    return module_path, namespaced_type.name


def generate_init_py(directory, package_name, interface_type, interface_names):
    """
    Generate __init__.py files following ROS 2 convention.

    Args:
        directory (str): Directory to create/update __init__.py in
        package_name (str): Name of the ROS package
        interface_type (str): Type of interface (msg, srv, action)
        interface_names (list): List of interface names to import

    Returns:
        str: Path to the generated/updated __init__.py file
    """
    init_file = os.path.join(directory, "__init__.py")
    
    # Create the directory if it doesn't exist
    os.makedirs(os.path.dirname(init_file), exist_ok=True)
    
    # Check if init file exists and read existing content
    existing_imports = []
    if os.path.exists(init_file):
        with open(init_file, "r", encoding="utf-8") as f:
            existing_imports = f.readlines()
    
    # Generate imports in ROS 2 format
    imports = []
    for name in sorted(interface_names):
        # Convert CamelCase to snake_case
        snake_case_name = "_".join(re.findall(r"[a-z]+|[A-Z][a-z]*", name)).lower()
        # Create import line in ROS 2 standard format
        import_line = (
            f"from {package_name}.{interface_type}._"
            f"{snake_case_name}_pb_support import convert_to_proto, convert_to_ros  # noqa: F401\n"
        )
        
        # Only add if not already present
        if import_line not in existing_imports:
            imports.append(import_line)
    
    # Write or append to the init file
    if imports or not os.path.exists(init_file):
        with open(init_file, "w", encoding="utf-8") as f:
            # Write a header
            f.write("# Generated by rosidl_converter_protobuf_py\n\n")
            
            # Preserve existing imports and comments
            for line in existing_imports:
                if "Generated by rosidl_converter_protobuf_py" not in line:
                    f.write(line)
            
            # Add a newline if needed
            if existing_imports and not existing_imports[-1].endswith("\n"):
                f.write("\n")
            
            # Add new imports
            f.writelines(imports)
    
    return init_file

def generate_py(generator_arguments_file):
    """
    Generate Python type support for Protocol Buffers.

    Args:
        generator_arguments_file: Path to the JSON file containing generator arguments
        
    Returns:
        int: Return code (0 for success, non-zero for errors)
    """
    with open(generator_arguments_file, "r", encoding="utf-8") as f:
        args = json.load(f)

    package_name = args["package_name"]
    interfaces = args.get("ros_interface_files", [])
    output_dir = args["output_dir"]
    template_dir = args.get("template_dir", "")
    idl_tuples = args.get("idl_tuples", [])

    logger.debug(f"Generating Python type support for package: {package_name}")
    logger.debug(f"Output directory: {output_dir}")
    logger.debug(f"Template directory: {template_dir}")
    logger.debug(f"IDL tuples: {idl_tuples}")
    logger.debug(f"Interfaces: {interfaces}")

    # Map to keep track of processed interfaces to avoid duplicates
    processed_interfaces = set()

    # Create output directory
    try:
        os.makedirs(output_dir, exist_ok=True)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    generated_files = []
    typesupport_impls = []

    # Process each IDL tuple
    for idl_tuple in idl_tuples:
        idl_parts = idl_tuple.split(":")
        if len(idl_parts) != 2:
            logger.warning("Invalid IDL tuple: %s", idl_tuple)
            continue

        # The first part is the package path, the second part contains interface type and name
        pkg_path, rel_path = idl_parts

        # Extract interface type from the relative path
        interface_type = None
        if rel_path.startswith("msg/"):
            interface_type = "msg"
        else:
            logger.warning("Only works for msg type and interface type for %s is currently being skipped so as to not lose my mind", rel_path)
            continue

        # Get the base name without the .idl extension
        idl_filename = os.path.basename(rel_path)
        idl_base_name = os.path.splitext(idl_filename)[0]

        logger.debug("Processing %s file: %s", interface_type, idl_base_name)

        # Check if already processed
        if idl_base_name in processed_interfaces:
            logger.debug("Skipping already processed interface: %s", idl_base_name)
            continue

        processed_interfaces.add(idl_base_name)

        # Get the full IDL file path
        idl_file = os.path.join(pkg_path, rel_path)

        # Check if the IDL file exists
        if not os.path.exists(idl_file):
            logger.warning("IDL file not found: %s", idl_file)

            # Create output directories
            output_subdir = os.path.join(output_dir, interface_type)
            os.makedirs(output_subdir, exist_ok=True)

            # Create placeholder file
            snake_case_name = "_".join(re.findall(r"[a-z]+|[A-Z][a-z]*", idl_base_name)).lower()
            placeholder_file = os.path.join(output_subdir, f"{snake_case_name}_pb_support.py")

            with open(placeholder_file, "w", encoding="utf-8") as f:
                f.write("# Generated by rosidl_converter_protobuf_py (placeholder)\n\n")
                f.write(f"# IDL file not found: {idl_file}\n")
                f.write("def convert_to_proto(ros_msg, pb_msg):\n")
                f.write("    return False\n\n")
                f.write("def convert_to_ros(pb_msg, ros_msg):\n")
                f.write("    return False\n")

            generated_files.append(placeholder_file)
            continue

        try:
            from rosidl_parser.definition import IdlLocator, Message
            from rosidl_parser.parser import parse_idl_file

            locator = IdlLocator(pkg_path, rel_path)
            # Call the function with the correct parameters
            logger.error("the pkg path and rel path is: %s %s", pkg_path, rel_path)
            logger.error("the locator is: %s", locator)
            content = parse_idl_file(locator)  # No package_name parameter

            logger.error("the contents of the content is %s: ", content)
            logger.error("the interface type is %s: ", interface_type)
            # Extract the appropriate content based on the interface type
            if interface_type == "msg":
                msg_elements = [elem for elem in content.content.elements if isinstance(elem, Message)]
                logger.error("the msg elements are %s: ", msg_elements)
                if msg_elements:
                    msg_content = msg_elements[0]
                    logger.error("the content is %s: ", msg_content)

            else:
                logger.warning("No content found in the IDL file %s", idl_file)

                # Create output directories
                output_subdir = os.path.join(output_dir, interface_type)
                os.makedirs(output_subdir, exist_ok=True)

                # Create placeholder file
                snake_case_name = "_".join(re.findall(r"[a-z]+|[A-Z][a-z]*", idl_base_name)).lower()
                placeholder_file = os.path.join(output_subdir, f"{snake_case_name}_pb_support.py")

                with open(placeholder_file, "w", encoding="utf-8") as f:
                    f.write("# Generated by rosidl_converter_protobuf_py (placeholder)\n\n")
                    f.write(f"# No message found in IDL file: {idl_file}\n")
                    f.write("def convert_to_proto(ros_msg, pb_msg):\n")
                    f.write("    return False\n\n")
                    f.write("def convert_to_ros(pb_msg, ros_msg):\n")
                    f.write("    return False\n")

                generated_files.append(placeholder_file)
                continue

        except (FileNotFoundError, ValueError, TypeError, ImportError) as e:
            logger.error("Error parsing %s: %s", idl_file, str(e))
            logger.debug(traceback.format_exc())

            # Create output directories
            output_subdir = os.path.join(output_dir, interface_type)
            os.makedirs(output_subdir, exist_ok=True)

            # Create placeholder file
            snake_case_name = "_".join(re.findall(r"[a-z]+|[A-Z][a-z]*", idl_base_name)).lower()
            placeholder_file = os.path.join(output_subdir, f"{snake_case_name}_pb_support.py")

            with open(placeholder_file, "w", encoding="utf-8") as f:
                f.write("# Generated by rosidl_converter_protobuf_py (placeholder)\n\n")
                f.write(f"# Error parsing IDL file: {str(e)}\n")
                f.write("def convert_to_proto(ros_msg, pb_msg):\n")
                f.write("    return False\n\n")
                f.write("def convert_to_ros(pb_msg, ros_msg):\n")
                f.write("    return False\n")

            generated_files.append(placeholder_file)
            continue

        # Determine the output file
        output_subdir = os.path.join(output_dir, interface_type)
        os.makedirs(output_subdir, exist_ok=True)

        # Convert CamelCase to snake_case for the file name
        snake_case_name = "_".join(re.findall(r"[a-z]+|[A-Z][a-z]*", idl_base_name)).lower()
        support_file = os.path.join(output_subdir, f"{snake_case_name}_pb_support.py")

        # Generate the typesupport file using the template
        try:
            # Prepare template variables
            data = {
                "package_name": package_name,
                "interface_path": idl_file,
                "content": content,
            }

            # Determine which template to use
            template_file = os.path.join(template_dir, "type_support.py.em")

            # Render the template
            with open(template_file, "r", encoding="utf-8") as template:
                template_text = template.read()

                # Pass to interpreter
                with open(support_file, "w", encoding="utf-8") as f:
                    interpreter = Interpreter(output=f, globals=data)
                    interpreter.file(StringIO(template_text))  # feed as file-like object
                    interpreter.shutdown()

            generated_files.append(support_file)

            # Record type support implementations for the init file
            interface_display_name = ""
            if interface_type == "msg":
                ros_type = ros_type_from_namespaced_type_py(
                    msg_content.structure.namespaced_type
                )
                proto_type = protobuf_type_from_namespaced_type_py(
                    msg_content.structure.namespaced_type
                )
                interface_display_name = msg_content.structure.namespaced_type.name
                typesupport_impls.append(
                    (interface_type, ros_type, proto_type, interface_display_name)
                )
        except (OSError, IOError, RuntimeError) as e:
            logger.error("Error generating %s: %s", support_file, str(e))
            logger.debug(traceback.format_exc())

            # Create a placeholder file
            with open(support_file, "w", encoding="utf-8") as f:
                f.write("# Generated by rosidl_converter_protobuf_py (placeholder)\n\n")
                f.write(f"# Error generating file: {str(e)}\n")
                f.write("def convert_to_proto(ros_msg, pb_msg):\n")
                f.write("    return False\n\n")
                f.write("def convert_to_ros(pb_msg, ros_msg):\n")
                f.write("    return False\n")

            generated_files.append(support_file)
            continue

    # Group type support implementations by type and generate __init__.py files
    for dir_type in ["msg"]:
        dir_path = os.path.join(output_dir, dir_type)

        # Skip if the directory doesn't exist
        if not os.path.exists(dir_path):
            continue

        # Skip if there are no implementations for this type
        type_specific_names = [
            impl[3] for impl in typesupport_impls if impl[0] == dir_type
        ]

        # Create an __init__.py file even if no implementations
        # This prevents installation errors
        generate_init_py(dir_path, package_name, dir_type, type_specific_names)

    # Generate the package __init__.py file
    package_init_file = os.path.join(output_dir, "__init__.py")
    with open(package_init_file, "w", encoding="utf-8") as f:
        f.write("# Generated by rosidl_converter_protobuf_py\n\n")

        # Generate imports by type
        for dir_type in ["msg"]:
            dir_path = os.path.join(output_dir, dir_type)
            if os.path.exists(dir_path):
                f.write(f"# {dir_type.capitalize()} types\n")
                f.write(f"from . import {dir_type}  # noqa: F401,F403\n\n")

        f.write("# End of generated imports\n")

    generated_files.append(package_init_file)

    # Log summary
    num_messages = len([impl for impl in typesupport_impls if impl[0] == "msg"])

    logger.info("Generated %d files:", len(generated_files))
    logger.info("  - %d message types", num_messages)

    return 0
