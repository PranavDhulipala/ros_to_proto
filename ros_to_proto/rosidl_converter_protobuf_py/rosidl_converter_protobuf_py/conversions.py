"""
Conversion registry and utility functions for ROS <-> Protobuf message conversion.

This module provides the infrastructure for registering and using converters 
between ROS 2 and Protocol Buffer message types.
"""

import logging
from typing import Dict, Callable, Any, Optional, TypeVar, Tuple, Set, Union, cast

# Type vars for typing
ROS = TypeVar("ROS")
PB = TypeVar("PB")

# Set up logging
logger = logging.getLogger("rosidl_converter_protobuf_py.conversions")

# Global converter registry
# Maps (ros_type, proto_type) to (to_proto_fn, to_ros_fn, serialize_fn, deserialize_fn)
_converters: Dict[
    Tuple[str, str],
    Tuple[
        Optional[Callable[[ROS, PB], bool]],
        Optional[Callable[[PB, ROS], bool]],
        Optional[Callable[[ROS], bytes]],
        Optional[Callable[[bytes], Optional[ROS]]]
    ]
] = {}

# Track registered primitive types
_primitive_types: Set[str] = set()

def register_converter(
    ros_type: str,
    proto_type: str,
    to_proto_fn: Optional[Callable[[ROS, PB], bool]],
    to_ros_fn: Optional[Callable[[PB, ROS], bool]],
    serialize_fn: Optional[Callable[[ROS], bytes]] = None,
    deserialize_fn: Optional[Callable[[bytes], Optional[ROS]]] = None,
) -> None:
    """
    Register conversion functions between a ROS type and Protocol Buffer type.

    Args:
        ros_type: Fully qualified ROS type (e.g., 'std_msgs/msg/String')
        proto_type: Fully qualified Protocol Buffer type (e.g., 'std_msgs.msg.String')
        to_proto_fn: Function that converts from ROS to Protocol Buffer
        to_ros_fn: Function that converts from Protocol Buffer to ROS
        serialize_fn: Optional function to serialize ROS message to bytes
        deserialize_fn: Optional function to deserialize bytes to ROS message
    """
    if ros_type and proto_type:
        _converters[(ros_type, proto_type)] = (
            to_proto_fn,
            to_ros_fn,
            serialize_fn,
            deserialize_fn,
        )
        logger.debug("Registered converter: %s <-> %s", ros_type, proto_type)


def convert(source: Any, target: Any, direction: str = "to_proto") -> bool:
    """
    Convert between ROS and Protocol Buffer types.

    Args:
        source: Source object (ROS or Protocol Buffer)
        target: Target object to populate (Protocol Buffer or ROS)
        direction: Either 'to_proto' or 'to_ros'

    Returns:
        True if conversion was successful, False otherwise
    """
    if source is None or target is None:
        return False

    # Get source type information
    ros_type = None
    proto_type = None
    
    if direction == "to_proto":
        # ROS message to Protocol Buffer
        if hasattr(source, "_type"):
            ros_type = source._type
        else:
            # Try to infer from class name
            try:
                ros_type = f"{source.__module__.rsplit('.', 2)[0]}/{source.__class__.__name__}"
            except (AttributeError, IndexError):
                logger.warning("Could not determine ROS type for %s", source)
                return False
    
    elif direction == "to_ros":
        # Protocol Buffer to ROS message
        if hasattr(source, "DESCRIPTOR"):
            proto_type = source.DESCRIPTOR.full_name
        else:
            # Try to infer from class name
            try:
                proto_type = f"{source.__module__}.{source.__class__.__name__}"
            except AttributeError:
                logger.warning("Could not determine Protocol Buffer type for %s", source)
                return False
    
    else:
        logger.error("Invalid conversion direction: %s", direction)
        return False

    # Find converter
    if direction == "to_proto":
        # Find by ROS type
        for (r_type, p_type), (to_proto, _, _, _) in _converters.items():
            if r_type == ros_type and to_proto is not None:
                return to_proto(source, target)
    else:  # to_ros
        # Find by Protocol Buffer type
        for (r_type, p_type), (_, to_ros, _, _) in _converters.items():
            if p_type == proto_type and to_ros is not None:
                return to_ros(source, target)
    
    if direction == "to_proto" and ros_type in _primitive_types:
        try:
            if hasattr(source, "data") and hasattr(target, "data"):
                target.data = source.data
                return True
        except Exception as e:
            logger.warning("Error in primitive conversion: %s", e)
    
    elif direction == "to_ros" and proto_type in _primitive_types:
        try:
            if hasattr(source, "data") and hasattr(target, "data"):
                target.data = source.data
                return True
        except Exception as e:
            logger.warning(f"Error in primitive conversion: {e}")
    
    # No converter found
    logger.warning("No converter found for %s: %s → %s", direction, ros_type or 'unknown', proto_type or 'unknown')
    return False


def register_primitive_conversion(ros_type: str, proto_type: str) -> None:
    """
    Register primitive type conversion (for primitive ROS types).

    Args:
        ros_type: ROS primitive type (e.g., 'std_msgs/msg/Int32')
        proto_type: Protocol Buffer type (e.g., 'std_msgs.msg.Int32')
    """
    # For primitive types, conversion is straightforward
    def to_proto(ros_msg: Any, pb_msg: Any) -> bool:
        """
        Convert from ROS message to Protocol Buffer message for primitive types.

        Args:
            ros_msg: ROS message
            pb_msg: Protocol Buffer message
    
        Returns:
            True if conversion was successful, False otherwise
        """
        try:
            if hasattr(ros_msg, "data") and hasattr(pb_msg, "data"):
                pb_msg.data = ros_msg.data
            return True
        except Exception as e:
            logger.warning("Error in primitive to_proto: %s", e)
            return False

    def to_ros(pb_msg: Any, ros_msg: Any) -> bool:
        """
        Convert from Protocol Buffer message to ROS message for primitive types.

        Args:
            pb_msg: Protocol Buffer message
            ros_msg: ROS message
        
        Returns:
            True if conversion was successful, False otherwise
        """
        try:
            if hasattr(pb_msg, "data") and hasattr(ros_msg, "data"):
                ros_msg.data = pb_msg.data
            return True
        except Exception as e:
            logger.warning("Error in primitive to_ros: %s", e)
            return False

    # Add to primitive types registry
    _primitive_types.add(ros_type)
    _primitive_types.add(proto_type)
    
    # Register the converter
    register_converter(ros_type, proto_type, to_proto, to_ros)


def get_converter(ros_type: str, proto_type: str) -> Optional[Tuple[Callable, Callable, Optional[Callable], Optional[Callable]]]:
    """
    Get the converter functions for a specific ROS and Protocol Buffer type pair.
    
    Args:
        ros_type: Fully qualified ROS type
        proto_type: Fully qualified Protocol Buffer type
        
    Returns:
        Tuple of (to_proto_fn, to_ros_fn, serialize_fn, deserialize_fn) or None if not found
    """
    return _converters.get((ros_type, proto_type))


def get_proto_type_for_ros(ros_type: str) -> Optional[str]:
    """
    Get the Protocol Buffer type registered for a ROS type.
    
    Args:
        ros_type: Fully qualified ROS type
        
    Returns:
        Protocol Buffer type or None if not found
    """
    for (r_type, p_type) in _converters.keys():
        if r_type == ros_type:
            return p_type
    return None


def get_ros_type_for_proto(proto_type: str) -> Optional[str]:
    """
    Get the ROS type registered for a Protocol Buffer type.
    
    Args:
        proto_type: Fully qualified Protocol Buffer type
        
    Returns:
        ROS type or None if not found
    """
    for (r_type, p_type) in _converters.keys():
        if p_type == proto_type:
            return r_type
    return None


def list_converters() -> Dict[Tuple[str, str], Tuple[Callable, Callable, Optional[Callable], Optional[Callable]]]:
    """
    Get a copy of the converter registry.
    
    Returns:
        Dictionary mapping (ros_type, proto_type) to converter functions
    """
    return _converters.copy()