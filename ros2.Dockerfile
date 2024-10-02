FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# launch ros package
# CMD ["ros2", "run", "demo_nodes_cpp", "listener"]
CMD ["ros2", "run", "demo_nodes_cpp", "talker"]