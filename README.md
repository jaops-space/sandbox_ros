# sandbox_ros

Sandbox for testing interfacing with ROS1 Noetic and ROS2 Foxy

Several containers are provided 
- ros1: has ROS1 Noetic installed. Launches a roscore + the talker and listener example (string with counter on topic /chatter)
- ros2: has ROS2 Foxy installed. launches the talker example (string with counter on topic /chatter)
- bridge: passes the topics both ways between ROS1 and ROS2 (see https://github.com/ros2/ros1_bridge/)
- ubuntu20: has Ubuntu 20.04 (focal) installed with ROS2 foxy. For example, access its shell with `docker exec -it sandbox_ros-ubuntu20-1 /bin/bash`


## Example workflow

1. Launch all the containers with `docker compose -f ./docker-compose.yml up`

(or select specific services to run with for example  `docker compose -f ./docker-compose.yml up ros2 ubuntu20`)

2. in another terminal, access the shell of the ubuntu20 container `docker exec -it sandbox_ros-ubuntu20-1 /bin/bash`
(the name might be different. check with `docker ps`)

3. within the ubuntu20 container console: `source /opt/ros/foxy/setup.bash` and `ros2 topic list`