# Use Ubuntu 22.04 as the base image (required for ROS 2 humble)
FROM ubuntu:22.04

# Set noninteractive mode to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Update and install required dependencies
RUN apt update && apt upgrade -y && \
    apt install -y locales curl wget gnupg lsb-release software-properties-common \
    build-essential cmake git python3 python3-pip python3-venv \
    libssl-dev libx11-dev libgl1-mesa-dev libgtest-dev \
    protobuf-compiler libprotobuf-dev && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Set the ROS 2 apt repository and keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install additional tools required for ROS 2 compilation
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-flake8-docstrings \
    python3-pytest-cov \
    python3-protobuf \
    ros-dev-tools \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures && \
    rm -rf /var/lib/apt/lists/*

# Install Eclipse Zenoh
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    curl && \
    ln -sf /bin/true /usr/bin/systemctl && \
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends zenoh && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a workspace for building ROS 2 from source
WORKDIR /opt/ros2_ws
RUN mkdir -p src && cd src && git clone -b humble https://github.com/ros2/ros2.git

# Modify ros2.repos to add rosidl_typesupport_protobuf
RUN sed -i '$ a \ \ ros2/rosidl_typesupport_protobuf:\n\ \ \ \ type: git\n\ \ \ \ url: https://github.com/eclipse-ecal/rosidl_typesupport_protobuf.git\n\ \ \ \ version: master' src/ros2/ros2.repos

# # Modify ros2.repos to add proto2ros
RUN sed -i '$ a \ \ ros2/proto2ros:\n\ \ \ \ type: git\n\ \ \ \ url: https://github.com/bdaiinstitute/proto2ros.git\n\ \ \ \ version: main' src/ros2/ros2.repos

RUN vcs import src < src/ros2/ros2.repos

# Install ROS 2 dependencies using rosdep
RUN apt update && \
    rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Replace the compile_proto function in the rosidl_adapter_proto package to compile protobuf for python as well
RUN perl -0777 -pi -e 's|def compile_proto\(protoc_path, proto_path_list, cpp_out_dir, proto_files, package_name\):(?:\n\s+.*?)*?subprocess\.check_call\(protoc_cmd\)|\
import os\n\
def compile_proto(protoc_path, proto_path_list, cpp_out_dir, proto_files, package_name):\n\
    proto_paths = [f"--proto_path={path}" for path in proto_path_list]\n\
    cpp_command = [\n\
        protoc_path, *proto_paths,\n\
        f"--cpp_out=dllexport_decl=ROSIDL_ADAPTER_PROTO_PUBLIC__{package_name}:{cpp_out_dir}",\n\
        *proto_files\n\
    ]\n\
    python_command = [\n\
        protoc_path, *proto_paths,\n\
        f"--python_out={cpp_out_dir}",\n\
        *proto_files\n\
    ]\n\
    def run_command(cmd):\n\
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)\n\
        for line in process.stdout:\n\
            print(line, end="")\n\
        stderr_output = process.stderr.read()\n\
        if stderr_output:\n\
            print(f"Error:{stderr_output}")\n\
        return process.wait()\n\
    if run_command(cpp_command) != 0:\n\
        print("Failed to generate C++ Protobuf")\n\
        return 1\n\
    if run_command(python_command) != 0:\n\
        print("Failed to generate Python Protobuf")\n\
        return 1\n\
    return 0|gs' /opt/ros2_ws/src/ros2/rosidl_typesupport_protobuf/rosidl_adapter_proto/rosidl_adapter_proto/__init__.py

# Add the following lines to the end of the file of the cmake file to install the generated Python Protobuf files in the correct location
RUN printf '\n\
\n\
execute_process(\n\
  COMMAND python3 -c "import sys; print(f'\''{sys.version_info.major}.{sys.version_info.minor}'\'')"\n\
  OUTPUT_VARIABLE PYTHON_VERSION\n\
  OUTPUT_STRIP_TRAILING_WHITESPACE\n\
)\n\
\n\
install(\n\
  DIRECTORY "${CMAKE_BINARY_DIR}/rosidl_adapter_proto/${PROJECT_NAME}/msg/"\n\
  DESTINATION "local/lib/python${PYTHON_VERSION}/dist-packages/${PROJECT_NAME}/msg"\n\
  FILES_MATCHING PATTERN "*.py"\n\
)\n\
\n\
install(\n\
  DIRECTORY "${CMAKE_BINARY_DIR}/rosidl_adapter_proto/${PROJECT_NAME}/srv/"\n\
  DESTINATION "local/lib/python${PYTHON_VERSION}/dist-packages/${PROJECT_NAME}/srv"\n\
  FILES_MATCHING PATTERN "*.py"\n\
)\n\
\n\
install(\n\
  DIRECTORY "${CMAKE_BINARY_DIR}/rosidl_adapter_proto/${PROJECT_NAME}/action/"\n\
  DESTINATION "local/lib/python${PYTHON_VERSION}/dist-packages/${PROJECT_NAME}/action"\n\
  FILES_MATCHING PATTERN "*.py"\n\
)\n\
\n' >> /opt/ros2_ws/src/ros2/rosidl_typesupport_protobuf/rosidl_adapter_proto/cmake/rosidl_adapt_proto_interfaces.cmake

# Build ROS 2 from source
RUN colcon build --parallel-workers $(nproc) --symlink-install --executor parallel

# For now we are just copying the packages into home/test_ws
# ideally the rosidl_converter_protobuf_py would be pulled as a part of the ros build
# like how we are dealing with rosidl_typesupport_protobuf and proto2ros

# Create test workspace directory structure
RUN mkdir -p /home/test_ws/src

COPY ros_to_proto/ /home/test_ws/src/

WORKDIR /home/test_ws
RUN bash -c "source /opt/ros2_ws/install/local_setup.bash && \
    colcon build --parallel-workers $(nproc) --symlink-install --executor parallel"

# Source ROS 2 setup script by default
RUN echo "source /opt/ros2_ws/install/local_setup.bash" >> /root/.bashrc
RUN echo "source /home/test_ws/install/local_setup.bash" >> /root/.bashrc

# Set default entrypoint to the bash shell
CMD ["/bin/bash"]
