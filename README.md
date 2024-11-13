# Lidar
This repository contains all the code for the LiDAR sensor, as well as how to launch and configure it

# Installation
Please follow the instructions on [GitHub](https://github.com/Livox-SDK/livox_ros_driver2) to install the driver for the Mid-360 LiDAR sensor

# Starting the LiDAR Sensor in ROS

1. Open a new terminal window.
2. Type the command `ros2 launch livox_ros_driver2 msg_MID360_launch.py` (without the quotation marks).
   - Alternatively, use `ros2 launch livox_ros_driver2 rviz_MID360_launch.py` if you want a visualization of the LiDAR sensor.
3. To check for topics, type `ros2 topic list`; you should see `/scan` as a topic.
4. To view nodes, type `rqt`; you should see a node named `/livox/lidar` with an arrow pointing to the `/scan` topic.

## For the A2M8 Model
- Use `ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py` for the viewer.
- Use `ros2 launch sllidar_ros2 sllidar_a2m8_launch.py` to view just the topic.

## Configuration
To edit the configuration file (or before building):
- File path after building: `/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
- File path before building: `/ws_livox/config`

You can also edit configuration files to change LiDAR parameters in:
- `/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/`
