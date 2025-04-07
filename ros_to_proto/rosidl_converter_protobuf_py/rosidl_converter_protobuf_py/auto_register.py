"""
Auto-register ROS <-> Protobuf converters.

This module detects and registers all available ROS to Protocol Buffer converters
to enable automatic conversion between message types.
"""

import os
import sys
import logging
import importlib
from typing import List

logger = logging.getLogger("rosidl_converter_protobuf_py.auto_register")

def find_converter_modules() -> List[str]:
    """
    Find all potential ROS <-> Protobuf converter modules in the Python path.
    
    Returns:
        List of module names that could contain converters
    """
    converter_modules = []
    
    # Search for modules with _pb_support patterns
    for path in sys.path:
        if not os.path.isdir(path):
            continue
            
        # Scan directories for ROS packages
        for pkg_name in os.listdir(path):
            pkg_path = os.path.join(path, pkg_name)
            
            # Skip non-directories and hidden directories
            if not os.path.isdir(pkg_path) or pkg_name.startswith('.'):
                continue
                
            # Check for msg, srv, action directories
            for interface_type in ['msg', 'srv', 'action']:
                interface_path = os.path.join(pkg_path, interface_type)
                
                if not os.path.isdir(interface_path):
                    continue
                    
                # Find *_pb_support.py files
                for filename in os.listdir(interface_path):
                    if filename.endswith('_pb_support.py'):
                        module_name = f"{pkg_name}.{interface_type}.{filename[:-3]}"
                        converter_modules.append(module_name)
    
    return converter_modules


def register_converters_in_module(module_name: str) -> int:
    """
    Import a module and register any converters it contains.
    
    Args:
        module_name: Name of the module to import
        
    Returns:
        Number of converters registered
    """
    try:
        # Import the module
        module = importlib.import_module(module_name)
        
        # Find convert_to_proto and convert_to_ros functions
        convert_functions = []
        for attr_name in dir(module):
            if attr_name.startswith('convert_') and (
                attr_name.endswith('_to_proto') or attr_name.endswith('_to_ros')):
                convert_functions.append(attr_name)
        
        # Import directly registers the converters, so just count them
        return len(convert_functions) // 2  # Divide by 2 since each converter has to_proto and to_ros
        
    except ImportError as e:
        logger.warning("Could not import converter module %s: %s", module_name, e)
        return 0
    except Exception as e:
        logger.error("Error registering converters in module %s: %s", module_name, e)
        return 0


def auto_register_all() -> int:
    """
    Find and register all available ROS <-> Protobuf converters.
    
    Returns:
        Number of converters registered
    """
    converter_modules = find_converter_modules()
    total_registered = 0
    
    for module_name in converter_modules:
        num_registered = register_converters_in_module(module_name)
        total_registered += num_registered
        
    logger.info("Auto-registered %d ROS <-> Protobuf converters", total_registered)
    return total_registered


# Automatically register converters when this module is imported
if __name__ != "__main__":
    auto_register_all()