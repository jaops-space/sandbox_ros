ARG ROS_DISTRO=foxy
FROM ros:$ROS_DISTRO as base
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Rust and the cargo-ament-build plugin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.74.0 -y
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install cargo-ament-build

RUN pip install --upgrade pytest 

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

RUN mkdir -p /workspace && echo "Did you forget to mount the repository into the Docker container?" > /workspace/HELLO.txt

RUN cd /workspace && git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust && cd src/ros2_rust && git checkout 047f48387adde48ed55b69506d43c0098ab3caad 
RUN cd /workspace && vcs import src < src/ros2_rust/ros2_rust_foxy.repos && . /opt/ros/foxy/setup.sh && colcon build

WORKDIR /workspace
