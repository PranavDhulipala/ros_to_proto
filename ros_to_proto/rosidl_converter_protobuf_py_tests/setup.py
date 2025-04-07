from setuptools import find_packages, setup

package_name = 'rosidl_converter_protobuf_py_tests'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pranav",
    maintainer_email="p.dhulipala@reply.de",
    description="ROS-Protobuf message conversion utilities",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "conversion_tests = rosidl_converter_protobuf_py_tests.test_data_acquisiton_conversion:main",
        ],
    },
)
