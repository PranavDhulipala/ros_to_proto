@{  
# --------------------------------------------------------------------------
# Pre-header logic and utility methods
# --------------------------------------------------------------------------
import logging
import re
from typing import (
    Any,
    List,
    Tuple
)

from rosidl_parser.definition import *

logger = logging.getLogger("rosidl_converter_protobuf_py_em")


def ros_type_from_namespaced_type_py(namespaced_type):
    """
    Convert a NamespacedType to a Python ROS message type string.

    Args:
        namespaced_type: A NamespacedType object containing namespaces and name attributes

    Returns:
        str: A fully qualified ROS message type as a dot-separated string

    Example:
        If namespaced_type.namespaces = ['data_acquisition_interfaces', 'msg']
        and namespaced_type.name = 'DataAcquisitionData',
        the returned value will be:
            'data_acquisition_interfaces.msg.DataAcquisitionData'
    """
    return f"{'.'.join(namespaced_type.namespaces)}.{namespaced_type.name}"

def extract_message_name(fully_qualified_type: str) -> str:
    """
    Extract the package name from a fully qualified type string.

    Args:
        fully_qualified_type: A fully qualified message type (e.g., 'package.msg.MessageType')

    Returns:
        The package name (first component of the dotted path)

    Example:
        Input: 'data_acquisition_interfaces.msg.DataAcquisitionData'
        Output: 'data_acquisition_interfaces'
    """
    return fully_qualified_type.split(".")[0]


def protobuf_type_from_namespaced_type_py(namespaced_type):
    """
    Convert a NamespacedType to the Python module and class name for a Protobuf message.
    This function assumes that the generated protobuf module for the message is named by
    appending '_pb2' to the message name.

    Args:
        namespaced_type: A NamespacedType object containing namespaces and name attributes

    Returns:
        tuple: A tuple containing the module path and class name for the Protobuf message

        - module_path: The fully qualified module path for the Protobuf message
        - class_name: The class name of the Protobuf message
    
    Example:
        If namespaced_type.namespaces = ['data_acquisition_interfaces', 'msg']
        and namespaced_type.name = 'DataAcquisitionData',
        the returned tuple will be:
            ('data_acquisition_interfaces.msg.DataAcquisitionData_pb2', 'DataAcquisitionData')
    """
    module_path = f"{'.'.join(namespaced_type.namespaces)}.{namespaced_type.name}_pb2"
    return module_path, namespaced_type.name

def get_ros_proto_info(msg):
    """
    Get ROS and Protobuf type information for a message in a Python context.

    Args:
        msg: A ROS IDL message definition object with a structure attribute

    Returns:
        dict: A dictionary containing type information with the following keys:
            - ros_type: Fully qualified ROS message type
            - proto_type: Fully qualified Protobuf message type
            - ros_ns: List of ROS namespace components
            - ros_msg: ROS message name
            - proto_ns: List of Protobuf namespace components
            - proto_msg: Protobuf message name
    """

    ros_type = ros_type_from_namespaced_type_py(msg.structure.namespaced_type)
    proto_module, proto_class = protobuf_type_from_namespaced_type_py(msg.structure.namespaced_type)

    proto_type = f"{proto_module}.{proto_class}"

    ros_parts = ros_type.split(".")
    ros_ns = ros_parts[:-1]
    ros_msg = ros_parts[-1]

    proto_parts = proto_type.split(".")
    proto_ns = proto_parts[:-1]
    proto_msg = proto_parts[-1]

    return {
        "ros_type": ros_type,
        "proto_type": proto_type,
        "ros_ns": ros_ns,
        "ros_msg": ros_msg,
        "proto_ns": proto_ns,
        "proto_msg": proto_msg,
    }

def to_snake_case(name: str) -> str:
    """
    Convert a CamelCase string to snake_case.

    Args:
        name: A string in CamelCase format

    Returns:
        str: The input string converted to snake_case
    """
    s1 = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def import_path(ns: List[str]) -> str:
    """
    Convert a list of namespaces into a dotted import path.
    
    Args:
        ns: List of namespace components
        
    Returns:
        Dotted import path
    """
    return ".".join(ns)

def is_map_type(field_type: Any) -> bool:
    """
    Check if the field type is a map type.

    Args:
        field_type: Field type to check

    Returns:
        bool: True if the field type is a map type, False otherwise
    """
    return isinstance(field_type, AbstractNestedType) and hasattr(field_type, 'key_type')

def get_map_types(field_type: Any) -> Tuple[Any, Any]:
    """
    Get the key and value types for a map field.
    
    Args:
        field_type: Map field type
        
    Returns:
        Tuple of (key_type, value_type)
    """
    if not is_map_type(field_type):
        return None, None
    return field_type.key_type, field_type.value_type

def process_nested_type(value_type, field_name, nested_imports):
    """
    Process a nested type and add its import info to nested_imports if needed.

    Args:
        value_type: A NamespacedType representing a nested message type
        field_name: Name of the field containing this nested type
        nested_imports: List to which the import information will be appended

    Returns:
        None: This function modifies the nested_imports list in-place

    Note:
        Google protobuf types are skipped and not added to nested_imports
    """
    ros_nested_type = ros_type_from_namespaced_type_py(value_type)
    proto_module, proto_class = protobuf_type_from_namespaced_type_py(value_type)
    proto_nested_type = f"{proto_module}.{proto_class}"

    if "google.protobuf" in ros_nested_type:
        logger.debug("Skipping Google protobuf nested type: %s", ros_nested_type)
        return

    ros_nested_ns = ros_nested_type.split(".")[:-1]
    ros_nested_msg = ros_nested_type.split(".")[-1]
    proto_nested_ns = proto_nested_type.split(".")[:-1]
    proto_nested_msg = proto_nested_type.split(".")[-1]

    import_info = {
        "ros_nested_type": ros_nested_type,
        "proto_nested_type": proto_nested_type,
        "ros_nested_ns": ros_nested_ns,
        "ros_nested_msg": ros_nested_msg,
        "proto_nested_ns": proto_nested_ns,
        "proto_nested_msg": proto_nested_msg,
        "ros_name": field_name,
    }

    if import_info not in nested_imports:
        logger.debug("Adding nested import: %s", import_info)
        nested_imports.append(import_info)

def generate_message_conversion(message):
    """
    Generate conversion metadata for a specific ROS message type to its Protobuf equivalent.

    Args:
        message: A ROS IDL message definition object

    Returns:
        dict: Metadata dictionary with the following keys:
            - msg_info: Dictionary with ROS and Protobuf type information
            - nested_imports: List of nested message types that need to be imported
            - message: Original message object

    Note:
        This function analyzes all members of the message to find nested types
        and builds information needed for generating conversion code.
    """

    msg_info = get_ros_proto_info(message)
    logger.debug("Retrieved ROS-Protobuf info: %s", msg_info)

    nested_imports = []

    for member in message.structure.members:
        t = member.type
        logger.debug("Processing member for nested types: %s of type %s", member.name, t)

        if is_map_type(t):
            _, value_type = get_map_types(t)
            if isinstance(value_type, NamespacedType):
                process_nested_type(value_type, member.name, nested_imports)
            continue

        is_array = isinstance(t, AbstractSequence)
        value_type = t.value_type if is_array else t

        if isinstance(value_type, NamespacedType):
            process_nested_type(value_type, member.name, nested_imports)

    logger.debug("Final nested imports: %s", nested_imports)

    result = {
        "msg_info": msg_info,
        "nested_imports": nested_imports,
        "message": message,
    }

    return result


def process_idl_file(idl_file):
    """
    Process an IdlFile by extracting message definitions from its content.

    Args:
        idl_file: An instance of IdlFile containing an attribute 'content' with an 'elements' list.

    Returns:
        A list of message definitions extracted from the IdlFile.
    """
    messages_to_process = []

    if isinstance(idl_file, IdlFile) and hasattr(idl_file, "content"):
        content = idl_file.content
        if hasattr(content, "elements"):
            for definition in content.elements:
                if isinstance(definition, Message):
                    logger.debug("Found Message definition: %s", definition)
                    messages_to_process.append(definition)
                elif isinstance(definition, Service):
                    logger.debug("Found Service definition: %s", definition)
                    messages_to_process.append(definition.request_message)
                    messages_to_process.append(definition.response_message)
                elif isinstance(definition, Action):
                    logger.debug("Found Action definition: %s", definition)
                    messages_to_process.extend([
                        definition.goal,
                        definition.result,
                        definition.feedback,
                        definition.send_goal_service.request_message,
                        definition.send_goal_service.response_message,
                        definition.get_result_service.request_message,
                        definition.get_result_service.response_message,
                        definition.feedback_message
                    ])
        else:
            logger.debug("idl_file.content does not have an 'elements' attribute.")
    else:
        logger.debug("Provided object is not a valid IdlFile with a 'content' attribute.")

    return messages_to_process
}@

@{
messages_to_process = process_idl_file(content)
logger.debug("Processed messages: %s", messages_to_process)

message_codes = []
for msg in messages_to_process:
    message_codes.append(generate_message_conversion(msg))
logger.debug("Message codes: %s", message_codes)

msg_info = message_codes[0]["msg_info"] if message_codes else {}
logger.debug("Primary message info: %s", msg_info)

if message_codes and msg_info:
    ros_import_path = import_path(msg_info["ros_ns"])
    proto_import_path = import_path(msg_info["proto_ns"])
    ros_msg_name = msg_info.get("ros_msg", "")
    proto_msg_name = msg_info.get("proto_msg", "")
else:
    ros_import_path = ""
    proto_import_path = ""
    ros_msg_name = ""
    proto_msg_name = ""

def generate_list_of_messages_nested_code():
    """
    Generate code for handling nested message types in repeated fields.

    Iterates through all message codes and their nested imports, creating
    conditional type handling code for each nested message type.

    Returns:
        str: Python code as a string for handling nested message types in lists
    """
    result = ""
    for code in message_codes:
        if code.get("nested_imports"):
            for import_info in code["nested_imports"]:
                result += f"""
                # Handle nested message type: {import_info["ros_nested_msg"]}
                if isinstance(ros_item, {import_info["ros_nested_msg"]}RosType):
                    # Create a new protobuf message of the corresponding type
                    pb_item = {import_info["proto_nested_msg"]}PbType()
                    
                    # Convert using the appropriate conversion function
                    convert_{to_snake_case(import_info["ros_nested_msg"])}_to_proto(ros_item, pb_item)
                    
                    # Add the converted protobuf message to the repeated field
                    pb_repeated_field.append(pb_item)
"""
    return result

def generate_nested_message_code():
    """
    Generate code for handling nested message types in regular fields.

    Iterates through all message codes and their nested imports, creating
    conditional type handling code for converting individual nested message fields.

    Returns:
        str: Python code as a string for handling nested message types
    """
    result = ""
    for code in message_codes:
        if code.get("nested_imports"):
            for import_info in code["nested_imports"]:
                result += f"""
            # Handle nested message type: {import_info["ros_nested_msg"]}
            if isinstance(ros_value, {import_info["ros_nested_msg"]}RosType):
                # Create a new protobuf message of the corresponding type
                pb_nested = {import_info["proto_nested_msg"]}PbType()
                
                # Convert the nested message
                convert_{to_snake_case(import_info["ros_nested_msg"])}_to_proto(ros_value, pb_nested)
                
                # Set the converted message to the field
                setattr(pb_msg, pb_field_name, pb_nested)
                return
"""
    return result
}@
import re
import numpy as np
import array
from typing import Any, Callable, Dict,Optional, Tuple

@[if message_codes and ros_import_path and ros_msg_name]
# Import ROS and Protobuf message types
from @(ros_import_path) import @(ros_msg_name) as @(ros_msg_name)RosType
from @(proto_import_path) import @(proto_msg_name) as @(proto_msg_name)PbType
@[end if]

# Import conversions registry
from rosidl_converter_protobuf_py.conversions import register_converter
import logging

# ------------------------------------------------------------------------------
# Import nested message types and their converters if any
# ------------------------------------------------------------------------------
@[for code in message_codes]@
@[if code["nested_imports"]]@
@[for import_info in code["nested_imports"]]@
# Import types for nested message @(import_info["ros_nested_msg"])
from @(import_path(import_info["ros_nested_ns"])) import @(import_info["ros_nested_msg"]) as @(import_info["ros_nested_msg"])RosType
from @(import_path(import_info["proto_nested_ns"])) import @(import_info["proto_nested_msg"]) as @(import_info["proto_nested_msg"])PbType
from @(import_path(import_info["ros_nested_ns"])).@(to_snake_case(import_info["proto_nested_msg"]))_pb_support import convert_to_proto as convert_@(to_snake_case(import_info["ros_nested_msg"]))_to_proto
from @(import_path(import_info["ros_nested_ns"])).@(to_snake_case(import_info["proto_nested_msg"]))_pb_support import convert_to_ros as convert_@(to_snake_case(import_info["ros_nested_msg"]))_to_ros
@[end for]@
@[end if]@
@[end for]@

logger = logging.getLogger("@(extract_message_name(ros_import_path))_rosidl_converter_protobuf_py")

# Mapping from ROS base type to a NumPy dtype
NUMPY_DTYPE_MAPPING = {
    "boolean": np.bool_,
    "int8": np.int8,
    "uint8": np.uint8,
    "int16": np.int16,
    "uint16": np.uint16,
    "int32": np.int32,
    "uint32": np.uint32,
    "int64": np.int64,
    "uint64": np.uint64,
    "float": np.float32,
    "double": np.float64,
    "string": np.str_,
    "wstring": np.str_,
}

# Mapping from ROS base type to an array.array typecode
ARRAY_TYPECODE_MAPPING = {
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float": "f",
    "double": "d",
}

def convert_primitive(x: Any, base_type: str, bounded_length: Optional[int] = None) -> Any:
    """
    Convert a single value x according to the base_type.
    If bounded_length is provided (for bounded strings), the resulting string is truncated.
    
    Args:
        x: The value to convert
        base_type: The ROS base type string (e.g., "int32", "string", etc.)
        bounded_length: For bounded strings, the maximum allowed length
        
    Returns:
        The converted value matching the ROS type
    """
    if x is None:
        # Handle None values based on type
        if base_type == "boolean":
            return False
        elif base_type in ("int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"):
            return 0
        elif base_type in ("float", "double"):
            return 0.0
        elif base_type == "octet":
            return bytes([0])
        elif base_type in ("string", "wstring"):
            return ""
        else:
            return None
    
    # Convert based on type
    if base_type == "boolean":
        return bool(x)
    elif base_type in ("int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"):
        try:
            return int(x)
        except (TypeError, ValueError) as e:
            logger.warning(f"Failed to convert {x} to {base_type}: {e}. Using default 0.")
            return 0
    elif base_type in ("float", "double"):
        try:
            return float(x)
        except (TypeError, ValueError) as e:
            logger.warning(f"Failed to convert {x} to {base_type}: {e}. Using default 0.0.")
            return 0.0
    elif base_type == "octet":
        # Handle various input types for octet
        if isinstance(x, int):
            return bytes([x & 0xFF])
        elif isinstance(x, (bytes, bytearray)):
            if len(x) != 1:
                logger.warning(f"Expected single byte for octet, got {len(x)} bytes. Using first byte.")
            return x
        elif isinstance(x, str):
            # Try to convert string to byte
            try:
                # If it's a numeric string
                return bytes([int(x) & 0xFF])
            except (ValueError, TypeError):
                # If it's a character string
                if len(x) > 0:
                    return bytes([ord(x[0]) & 0xFF])
                return bytes([0])
        else:
            logger.warning(f"Unexpected type for octet: {type(x)}. Converting to bytes.")
            try:
                return bytes([int(x) & 0xFF])
            except (TypeError, ValueError):
                return bytes([0])
    elif base_type in ("string", "wstring"):
        # Handle string types
        if isinstance(x, bytes):
            try:
                # For wstring, try UTF-16 decoding first
                if base_type == "wstring":
                    try:
                        s = x.decode('utf-16')
                    except UnicodeDecodeError:
                        # Fall back to default string conversion
                        s = str(x, errors='replace')
                else:
                    # For regular string
                    s = str(x, errors='replace')
            except (TypeError, ValueError):
                s = str(x)
        else:
            s = str(x)
            
        # Apply bounds if necessary
        if bounded_length is not None and len(s) > bounded_length:
            logger.warning(f"Truncating {base_type} from length {len(s)} to {bounded_length}")
            s = s[:bounded_length]
        return s
    else:
        # Fallback: return as is
        return x

def parse_ros_type(ros_type_str: str) -> Tuple[str, Optional[int], Optional[int]]:
    """
    Parse a ROS field type string and extract the base type and any bounds/sizes.
    
    Args:
        ros_type_str: The ROS field type string (e.g., "int32", "string<10>", "int32[5]", etc.)
        
    Returns:
        Tuple of (base_type, array_size, bounded_length):
            - base_type: The base ROS type (e.g., "int32", "string", etc.)
            - array_size: Size for fixed-size arrays, None otherwise
            - bounded_length: Length bound for bounded strings or sequence bounds, None otherwise
            Note: For sequences, this represents the maximum allowed length (no padding)
    """
    # Fixed-size arrays: e.g., "int32[5]"
    fixed_array_match = re.match(r"^(.+)\[(\d+)\]$", ros_type_str)
    if fixed_array_match:
        base_type = fixed_array_match.group(1)
        array_size = int(fixed_array_match.group(2))
        return base_type, array_size, None
    
    # Bounded strings: e.g., "string<10>"
    bounded_str_match = re.match(r"^(string|wstring)<(\d+)>$", ros_type_str)
    if bounded_str_match:
        base_type = bounded_str_match.group(1)
        bounded_length = int(bounded_str_match.group(2))
        return base_type, None, bounded_length
    
    # Sequences: e.g., "sequence<int32>" or "sequence<int32, 5>"
    seq_match = re.match(r"^sequence<(.+)>$", ros_type_str)
    if seq_match:
        inner = seq_match.group(1).strip()
        if ',' in inner:
            parts = [p.strip() for p in inner.split(',')]
            base_type = parts[0]
            seq_bound = int(parts[1])
            # For sequences, bounded_length represents the maximum size (with no padding)
            return base_type, None, seq_bound
        else:
            base_type = inner
            return base_type, None, None
    
    # Basic type (no array/sequence/bound)
    return ros_type_str, None, None


def create_array_converter(base_type: str, target_container: type, fixed_size: Optional[int] = None, pad: bool = True) -> Callable:
    """
    Create a function to convert an array/sequence of values to the appropriate container type.
    
    Args:
        base_type: The ROS base type (e.g., "int32", "string", etc.)
        target_container: The Python container type (list, numpy.ndarray, array.array)
        fixed_size: For fixed-size arrays or bounded sequences, the expected/maximum size
        pad: Whether to pad arrays shorter than fixed_size (True for fixed arrays, False for bounded sequences)
        
    Returns:
        A function that converts input values to the appropriate container
    """
    def converter(values):
        """
        Convert the input values to the appropriate container type.

        Args:
            values: The input values to convert (can be None, a list, or an array-like object)

        Returns:
            The converted values in the specified container type
        """
        if values is None:
            if fixed_size is not None and pad:
                # Create a fixed-size array of default values
                raw = [convert_primitive(None, base_type) for _ in range(fixed_size)]
            else:
                # Empty sequence
                raw = []
        else:
            # Convert each element
            raw = list(values)
            
            # Handle size constraints
            if fixed_size is not None:
                if len(raw) > fixed_size:
                    array_type = "fixed-size array" if pad else "bounded sequence"
                    logger.warning(f"Truncating {array_type} from length {len(raw)} to {fixed_size}")
                    raw = raw[:fixed_size]
                elif pad and len(raw) < fixed_size:
                    # Pad with default values (only for fixed arrays, not bounded sequences)
                    raw.extend([None] * (fixed_size - len(raw)))
            
            # Convert each element to the appropriate type
            raw = [convert_primitive(x, base_type) for x in raw]
        
        # Return the appropriate container type
        if target_container is np.ndarray:
            np_dtype = NUMPY_DTYPE_MAPPING.get(base_type, np.str_)
            return np.array(raw, dtype=np_dtype)
        elif target_container is array.array:
            typecode = ARRAY_TYPECODE_MAPPING.get(base_type)
            if typecode:
                try:
                    return array.array(typecode, raw)
                except (TypeError, ValueError) as e:
                    logger.warning(f"Failed to create array.array: {e}. Falling back to list.")
                    return raw
            else:
                return raw
        else:
            # Default to list
            return raw
    
    return converter

def _get_proto_field_name(ros_field_name, field_name_mapping=None):
    """
    Map ROS field name to Protobuf field name using provided mapping or default convention

    Args:
        ros_field_name: The ROS field name
        field_name_mapping: Optional mapping from ROS field names to Protobuf field names.
                            If None, assumes field names are the same or follows a convention.

    Returns:
        The corresponding Protobuf field name
    """
    if field_name_mapping and ros_field_name in field_name_mapping:
        return field_name_mapping[ros_field_name]
    return ros_field_name  # Default: use the same field name


@{
###############################################################################
# convert_to_ros: Protobuf -> ROS for @(msg_info["ros_msg"])
###############################################################################
}@
def convert_to_ros(pb_msg: @(proto_msg_name)PbType, ros_msg: @(ros_msg_name)RosType, field_name_mapping: Optional[Dict[str, str]] = None) -> bool:
    """
    Convert a Protobuf message to a ROS message by examining the ROS field definitions.
    
    This function handles:
    - Single primitive fields (boolean, int, float, octet, string, wstring)
    - Bounded strings (e.g., "string<10>" or "wstring<5>")
    - Fixed-size arrays (e.g., "int8[4]")
    - Sequences (e.g., "sequence<int32>" or "sequence<int32, 5>")
    - Nested messages (messages that contain other messages)
    - Lists/sequences of nested messages
    
    Args:
        pb_msg: The source Protobuf message
        ros_msg: The target ROS message to populate
        field_name_mapping: Optional mapping from ROS field names to Protobuf field names.
                        If None, assumes field names are the same or follows a convention.
    
    Returns:
        True if conversion successful, False otherwise
    """
    if pb_msg is None or ros_msg is None:
        logger.error("Either pb_msg or ros_msg is None")
        return False

    def handle_nested_message(pb_value, ros_type_str):
        """
        Handle conversion of a nested Protobuf message to a ROS message

        Args:
            pb_value: The Protobuf message to convert
            ros_type_str: The ROS type string (e.g., "package_name/MessageType")

        Returns:
            The converted ROS message or None if conversion fails
        """
        try:
            # Extract nested message type name from the ROS type string
            # The type string will be something like "package_name/MessageType"
            if '/' in ros_type_str:
                # Handle older ROS1 style type strings
                nested_type_name = ros_type_str.split('/')[-1]
            else:
                # Handle newer ROS2 style type strings
                nested_type_name = ros_type_str.split('.')[-1]
            
            nested_pb_to_ros_func = None
            
@[for code in message_codes]@
@[if code["nested_imports"]]@
@[for import_info in code["nested_imports"]]@
            if nested_type_name == "@(import_info["ros_nested_msg"])":
                nested_pb_to_ros_func = convert_@(to_snake_case(import_info["ros_nested_msg"]))_to_ros
                # Create a new instance of the ROS message type
                ros_nested_type = @(import_info["ros_nested_msg"])RosType
                ros_nested = ros_nested_type()
                # Convert Protobuf to ROS
                nested_pb_to_ros_func(pb_value, ros_nested)
                return ros_nested
@[end for]@
@[end if]@
@[end for]@

            # If no specialized converter found, use generic convert_to_ros
            if not nested_pb_to_ros_func:
                logger.warning(f"No specialized converter found for nested message: {nested_type_name}")
                return None
        except Exception as e:
            logger.warning(f"Error converting nested message: {e}")
            return None

    def handle_list_of_messages(pb_list, ros_type_str):
        """
        Handle conversion of a list of Protobuf messages to a list of ROS messages

        Args:
            pb_list: The list of Protobuf messages to convert
            ros_type_str: The ROS type string (e.g., "sequence<package_name/MessageType>")
        
        Returns:
            A list of converted ROS messages or an empty list if conversion fails
        """
        try:
            # Extract the message type from the sequence type string
            # The type string will be something like "sequence<package_name/MessageType>"
            inner_type = ros_type_str.replace("sequence<", "").replace(">", "").split(",")[0].strip()
            
            result = []
            for pb_item in pb_list:
                ros_item = handle_nested_message(pb_item, inner_type)
                if ros_item:
                    result.append(ros_item)
            return result
        except Exception as e:
            logger.warning(f"Error converting list of nested messages: {e}")
            return []

    def is_message_type(type_str):
        """
        Check if a ROS type string represents a message (vs a primitive)

        Args:
            type_str: The ROS type string to check

        Returns:
            True if the type string represents a message, False otherwise
        """
        primitive_types = {"boolean", "int8", "uint8", "int16", "uint16", "int32", "uint32", 
                          "int64", "uint64", "float", "double", "string", "wstring", "octet"}
        
        # Remove array/sequence notation and bounds
        base_type = type_str
        if "[" in base_type:
            base_type = base_type.split("[")[0]
        if "<" in base_type:
            if "sequence<" in base_type:
                base_type = base_type.replace("sequence<", "").replace(">", "").split(",")[0].strip()
            else:
                base_type = base_type.split("<")[0]
        
        # Check if it's a primitive type
        return base_type not in primitive_types

    def safe_assign(ros_field_name: str, pb_field_name: str, convert_func: Callable, default_val: Any) -> None:
        """
        Safely assign a field value with proper error handling

        Args:
            ros_field_name: The ROS field name
            pb_field_name: The Protobuf field name
            convert_func: The conversion function to apply
            default_val: The default value to set in case of failure
        """
        try:
            # Check if the field exists in the Protobuf message
            if not hasattr(pb_msg, pb_field_name):
                logger.debug(f"Protobuf message missing field '{pb_field_name}'. "
                            f"Setting default for '{ros_field_name}'.")
                setattr(ros_msg, ros_field_name, default_val)
                return
            
            # Get the value and convert it
            pb_value = getattr(pb_msg, pb_field_name)
            converted_value = convert_func(pb_value)
            setattr(ros_msg, ros_field_name, converted_value)
        except Exception as e:
            logger.warning(f"Error converting field '{pb_field_name}' to '{ros_field_name}': {e}")
            setattr(ros_msg, ros_field_name, default_val)

    try:
        # Get all ROS message fields and their types
        fields_and_types = ros_msg.get_fields_and_field_types()
        
        # Process each field
        for ros_field_name, ros_type_str in fields_and_types.items():
            pb_field_name = _get_proto_field_name(ros_field_name, field_name_mapping)
            
            # Get the current value to determine container type
            target_value = getattr(ros_msg, ros_field_name)
            target_container = type(target_value)

            # Handling sequences of nested messages
            if "sequence<" in ros_type_str and is_message_type(ros_type_str):
                def convert_list_of_nested(pb_val):
                    return handle_list_of_messages(pb_val, ros_type_str)
                safe_assign(ros_field_name, pb_field_name, convert_list_of_nested, [])
                continue
            
            # Handling nested message types
            if is_message_type(ros_type_str):
                # Nested message type (not a primitive)
                def convert_nested(pb_val):
                    return handle_nested_message(pb_val, ros_type_str)
                safe_assign(ros_field_name, pb_field_name, convert_nested, None)
                continue
            
            # Parse the ROS type for non-nested types
            base_type, array_size, bounded_length = parse_ros_type(ros_type_str)
            
            if array_size is not None:
                # Fixed-size array
                converter = create_array_converter(base_type, target_container, array_size)
                default_val = converter(None)  # Create default array
                safe_assign(ros_field_name, pb_field_name, converter, default_val)
            
            elif "sequence" in ros_type_str:
                # Sequence (bounded or unbounded)
                if bounded_length is not None:
                    # Bounded sequence - truncate if too long but don't pad if too short
                    converter = create_array_converter(base_type, target_container, fixed_size=bounded_length, pad=False)
                else:
                    # Unbounded sequence
                    converter = create_array_converter(base_type, target_container)
                safe_assign(ros_field_name, pb_field_name, converter, [])
            
            elif bounded_length is not None:
                # Bounded string
                def convert_bounded_str(pb_val):
                    return convert_primitive(pb_val, base_type, bounded_length)
                safe_assign(ros_field_name, pb_field_name, convert_bounded_str, "")
            
            else:
                # Single primitive field
                def convert_single(pb_val):
                    return convert_primitive(pb_val, base_type)
                default_val = convert_primitive(None, base_type)
                safe_assign(ros_field_name, pb_field_name, convert_single, default_val)
        
        return True
    
    except Exception as e:
        logger.error(f"Error in convert_to_ros: {e}")
        return False

@{
###############################################################################
# convert_to_proto: ROS -> Protobuf for @(msg_info["ros_msg"])
###############################################################################
}@

def convert_to_proto(ros_msg: @(ros_msg_name)RosType, pb_msg: @(proto_msg_name)PbType, field_name_mapping: Optional[Dict[str, str]] = None) -> bool:
    """
    Convert a ROS message to a Protobuf message.
    
    This function handles:
    - Single primitive fields (boolean, int, float, octet, string, wstring)
    - Bounded strings
    - Fixed-size arrays and sequences
    - Nested messages (if the appropriate conversion function exists)
    
    Args:
        ros_msg: The source ROS message
        pb_msg: The target Protobuf message to populate
        field_name_mapping: Optional mapping from ROS field names to Protobuf field names.
                        If None, assumes field names are the same or follows a convention.
    
    Returns:
        True if conversion successful, False otherwise
    """
    if ros_msg is None or pb_msg is None:
        logger.error("Either ros_msg or pb_msg is None")
        return False

    def get_pb_field_info(pb_field_name):
        """
        Get Protobuf field type and whether it's repeated

        Args:
            pb_field_name: The Protobuf field name

        Returns:
            A dictionary with 'type' and 'is_repeated' keys
        """
        # Try to get field descriptor
        field_descriptor = pb_msg.DESCRIPTOR.fields_by_name.get(pb_field_name)
        if field_descriptor:
            return {
                'type': field_descriptor.type,
                'is_repeated': field_descriptor.label == field_descriptor.LABEL_REPEATED
            }
        
        # Fallback if descriptor not available
        if hasattr(type(pb_msg), pb_field_name):
            pb_field_desc = getattr(type(pb_msg), pb_field_name)
            return {
                'type': getattr(pb_field_desc, 'type', None),
                'is_repeated': hasattr(pb_field_desc, 'label') and pb_field_desc.label == 3  # 3 means LABEL_REPEATED
            }
        
        # Default values if no information available
        return {'type': None, 'is_repeated': False}

    def handle_array_type(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated):
        """
        Handle array-like ROS values (numpy arrays, array.array)

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_type: The Protobuf field type
            pb_field_is_repeated: Whether the Protobuf field is repeated
        """
        # For byte fields (type 12), convert to bytes
        if pb_field_type == 12:  # TYPE_BYTES
            # Check if we can convert directly to bytes
            if (isinstance(ros_value, np.ndarray) and ros_value.dtype in [np.int8, np.uint8, np.bool_]) or \
                (isinstance(ros_value, array.array) and ros_value.typecode in ['b', 'B']):
                setattr(pb_msg, pb_field_name, ros_value.tobytes())
                return
        
        # Convert to list for other operations
        value_list = ros_value.tolist() if isinstance(ros_value, np.ndarray) else list(ros_value)
        
        # Handle repeated fields
        if pb_field_is_repeated:
            pb_field = getattr(pb_msg, pb_field_name)
            del pb_field[:]  # Delete all elements - more compatible than clear()
            pb_field.extend(value_list)
        else:
            setattr(pb_msg, pb_field_name, value_list)

    def handle_bytes(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated):
        """Handle bytes/octet fields"""
        if pb_field_type == 12:  # TYPE_BYTES
            # Direct assignment for bytes fields
            setattr(pb_msg, pb_field_name, ros_value)
        elif len(ros_value) == 1:
            # Single octet as integer
            setattr(pb_msg, pb_field_name, ros_value[0])
        else:
            # Multiple bytes as list
            value_list = list(ros_value)
            if pb_field_is_repeated:
                pb_field = getattr(pb_msg, pb_field_name)
                del pb_field[:]
                pb_field.extend(value_list)
            else:
                setattr(pb_msg, pb_field_name, value_list)

    def handle_list(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated):
        """
        Handle list-like ROS values (list, numpy arrays, etc.)

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_type: The Protobuf field type
            pb_field_is_repeated: Whether the Protobuf field is repeated
        """
        if not ros_value:
            return  # Empty list - nothing to do
        
        # Handle list of bytes specially
        if isinstance(ros_value[0], bytes):
            handle_list_of_bytes(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated)
        
        # Handle list of nested messages
        elif hasattr(ros_value[0], 'get_fields_and_field_types'):
            handle_list_of_messages(ros_value, pb_field_name, pb_field_is_repeated)
        
        # String lists for byte fields
        elif pb_field_type == 12 and all(isinstance(x, str) for x in ros_value):
            handle_list_of_strings(ros_value, pb_field_name, pb_field_is_repeated)
        
        # Regular list of primitives
        elif pb_field_type == 12:  # TYPE_BYTES
            try:
                bytes_value = bytes(ros_value)
                setattr(pb_msg, pb_field_name, bytes_value)
            except (TypeError, ValueError) as e:
                logger.warning(f"Failed to convert list to bytes for field '{pb_field_name}': {e}")
        elif pb_field_is_repeated:
            # For repeated fields, clear and then extend
            try:
                pb_field = getattr(pb_msg, pb_field_name)
                del pb_field[:]
                pb_field.extend(ros_value)
            except TypeError as e:
                logger.warning(f"Failed to extend repeated field '{pb_field_name}': {e}")
        else:
            # Direct assignment for other fields
            try:
                setattr(pb_msg, pb_field_name, ros_value)
            except TypeError as e:
                logger.warning(f"Failed to set field '{pb_field_name}': {e}")

    def handle_list_of_bytes(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated):
        """
        Handle list of bytes for byte fields

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_type: The Protobuf field type
            pb_field_is_repeated: Whether the Protobuf field is repeated
        """
        if pb_field_type == 12:  # TYPE_BYTES
            # Try to concatenate bytes for byte fields
            combined_bytes = b''.join(ros_value)
            setattr(pb_msg, pb_field_name, combined_bytes)
        else:
            # Extract bytes as integers
            byte_list = [b[0] if len(b) == 1 else int.from_bytes(b, byteorder='little') for b in ros_value]
            if pb_field_is_repeated:
                pb_field = getattr(pb_msg, pb_field_name)
                del pb_field[:]
                pb_field.extend(byte_list)
            else:
                setattr(pb_msg, pb_field_name, byte_list)

    def handle_list_of_strings(ros_value, pb_field_name, pb_field_is_repeated):
        """
        Handle list of strings for wstring fields

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_is_repeated: Whether the Protobuf field is repeated
        """
        if pb_field_is_repeated:
            pb_field = getattr(pb_msg, pb_field_name)
            del pb_field[:]
            for string_item in ros_value:
                pb_field.append(string_item.encode('utf-16'))
        else:
            # For non-repeated fields, join all strings
            bytes_value = b''.join(x.encode('utf-16') for x in ros_value)
            setattr(pb_msg, pb_field_name, bytes_value)

    def handle_list_of_messages(ros_value, pb_field_name, pb_field_is_repeated):
        """
        Handle a list of nested ROS messages

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_is_repeated: Whether the Protobuf field is repeated
        """
        try:
            if not pb_field_is_repeated:
                logger.warning(f"Field '{pb_field_name}' is not a repeated field but ROS value is a list of messages")
                return

            # Get the repeated field from the protobuf message
            pb_repeated_field = getattr(pb_msg, pb_field_name)

            # Clear existing items
            del pb_repeated_field[:]

            # Check for None or empty list; nothing to convert if so
            if not ros_value:
                logger.debug(f"No items found in ROS value for field '{pb_field_name}'. Leaving it empty.")
                return

        @[if generate_list_of_messages_nested_code() != ""]
            # Process each message in the list
            for ros_item in ros_value:
            @(generate_list_of_messages_nested_code())
        @[end if]
        except Exception as e:
            logger.warning(f"Error converting nested message list in field '{pb_field_name}': {e}")

    def handle_nested_message(ros_value, pb_field_name):
        """
        Handle a single nested ROS message

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
        """
        try:
            # Check if ros_value is None
            if ros_value is None:
                logger.debug(f"No ROS value provided for field '{pb_field_name}'.")
                return
        @[if generate_nested_message_code() != ""]
            @(generate_nested_message_code())
        @[else]
                logger.warning(f"No converter found for nested message type: {type(ros_value).__name__}")
        @[end if]
        except Exception as e:
            logger.warning(f"Error converting nested message in field '{pb_field_name}': {e}")

    def handle_string(ros_value, pb_field_name, pb_field_type):
        """
        Handle string values

        Args:
            ros_value: The ROS value to convert
            pb_field_name: The Protobuf field name
            pb_field_type: The Protobuf field type
        """
        if pb_field_type == 12:  # TYPE_BYTES (for wstring)
            # Convert string to UTF-16 bytes for wstring fields
            setattr(pb_msg, pb_field_name, ros_value.encode('utf-16'))
        elif pb_field_type == 9:  # TYPE_STRING (for string)
            # Direct assignment for string fields
            setattr(pb_msg, pb_field_name, ros_value)
        else:
            # Try numeric conversion
            try:
                num_value = int(ros_value)
                setattr(pb_msg, pb_field_name, num_value)
            except ValueError:
                try:
                    num_value = float(ros_value)
                    setattr(pb_msg, pb_field_name, num_value)
                except ValueError:
                    # Just use as string
                    setattr(pb_msg, pb_field_name, ros_value)

    def safe_assign(ros_field_name, pb_field_name):
        """
        Safely assign a ROS field value to a Protobuf field

        Args:
            ros_field_name: The ROS field name
            pb_field_name: The Protobuf field name
        """
        try:
            # Check if the field exists in both messages
            if not hasattr(ros_msg, ros_field_name):
                logger.debug(f"ROS message missing field '{ros_field_name}'.")
                return
            
            if not hasattr(pb_msg, pb_field_name):
                logger.debug(f"Protobuf message doesn't have field '{pb_field_name}'.")
                return
            
            # Get the ROS value
            ros_value = getattr(ros_msg, ros_field_name)
            
            # Skip None values
            if ros_value is None:
                return
            
            # Get field information
            field_info = get_pb_field_info(pb_field_name)
            pb_field_type = field_info['type']
            pb_field_is_repeated = field_info['is_repeated']
            
            # Handle different types
            if isinstance(ros_value, (np.ndarray, array.array)):
                handle_array_type(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated)
            elif isinstance(ros_value, bytes):
                handle_bytes(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated)
            elif isinstance(ros_value, list):
                handle_list(ros_value, pb_field_name, pb_field_type, pb_field_is_repeated)
            elif hasattr(ros_value, 'get_fields_and_field_types'):
                handle_nested_message(ros_value, pb_field_name)
            elif isinstance(ros_value, str):
                handle_string(ros_value, pb_field_name, pb_field_type)
            else:
                # Direct assignment for primitives
                setattr(pb_msg, pb_field_name, ros_value)
                
        except Exception as e:
            logger.warning(f"Error converting field '{ros_field_name}' to '{pb_field_name}': {e}")

    try:
        # Get all ROS message fields
        fields_and_types = ros_msg.get_fields_and_field_types()
        
        # Process each field
        for ros_field_name in fields_and_types:
            pb_field_name = _get_proto_field_name(ros_field_name, field_name_mapping)
            safe_assign(ros_field_name, pb_field_name)
        
        return True
    
    except Exception as e:
        logger.error(f"Error in convert_to_proto: {e}")
        return False

# Register the converters with the conversion system
@{
###############################################################################
# Register converters
###############################################################################
}@

register_converter(
    "@(msg_info["ros_ns"][0])/@(ros_msg_name)",
    "@(msg_info["proto_ns"][0]).msg.pb.@(proto_msg_name)",
    convert_to_proto,
    convert_to_ros,
    serialize_fn=None,
    deserialize_fn=None 
)