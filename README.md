# sandbox_ros

Sandbox for testing interfacing with ROS1 Noetic and ROS2 Foxy

Several containers are provided :
- ros1: has ROS1 Noetic installed. Launches a roscore + the talker and listener example (string with counter on topic /chatter)
- ros2: has ROS2 Foxy installed. launches the talker example (string with counter on topic /chatter)
- bridge: passes the topics both ways between ROS1 and ROS2 (see https://github.com/ros2/ros1_bridge/)
- ubuntu20: has Ubuntu 20.04 (focal) installed with ROS2 foxy. For example, access its shell with `docker exec -it sandbox_ros-ubuntu20-1 /bin/bash`
- rust: has ros2_rust (https://github.com/ros2-rust/ros2_rust/blob/main/examples/minimal_pub_sub/src/minimal_publisher.rs / commit id 047f48387adde48ed55b69506d43c0098ab3caad) and ROS2 Foxy installed 


## Example Workflow A - test the ROS1/ROS2 bridge

1. Launch all the containers with `docker compose -f ./docker-compose.yml up`

(or select specific services to run, for example:  `docker compose -f ./docker-compose.yml up ros2 ubuntu20`)

2. In another terminal, access the shell of the ubuntu20 container `docker exec -it sandbox_ros-ubuntu20-1 /bin/bash`
(the name might be different. check with `docker ps`)

3. Within the ubuntu20 container terminal: `ros2 topic list`


## Example Workflow B - test the Rust ROS client with a basic ROS publisher

1. Launch ros2 and rust containers with `docker compose -f ./docker-compose.yml up ros2 rust`

2. In another terminal, access the shell of the ros2 container `docker exec -it sandbox_ros-ros2-1 /bin/bash`
(the name might be different. check with `docker ps`)

3. Within the ros2 container terminal: `ros2 topic pub /topic std_msgs/String "{data: 'test data'}" --rate 1`

4. In another terminal, access the shell of the rust container `docker exec -it sandbox_ros-rust-1 /bin/bash`
(the name might be different. check with `docker ps`)

5. Within the rust container terminal: `source ./install/setup.sh` and ` ros2 run examples_rclrs_minimal_pub_sub minimal_subscriber`


## Example Workflow C - test existing rosbag and Rust ROS client

1. Create a shared folder: `mkdir ws_ros2` and copy the rosbag into it.

2. Launch the ubuntu20 container with `docker compose -f ./docker-compose.yml up ubuntu20 rust` (the shared folder is mounted within the container)

3. In another terminal, access the shell of the container `docker exec -it sandbox_ros-ubuntu20-1 /bin/bash`
(the name might be different. check with `docker ps`)

4. Within the container terminal: `ros2 bag play /ws_ros2/path/to/bag`. It will publish all the messages in the rosbag at the appropriate timestamps.

5. In another terminal, access the shell of the rust container `docker exec -it sandbox_ros-rust-1 /bin/bash`

6. Within the container terminal, build the rust ROS workspace with `colcon build` (or the 'ccc' alias for just the 'examples_rclrs_minimal_pub_sub' package). Then run the customized rust programm 'sandbox_sub.rs' : `ros2 run examples_rclrs_minimal_pub_sub sandbox_sub`: it will print the messages from the rosbag to the terminal.
