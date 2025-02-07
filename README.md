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
### Driver Parameters
To edit the configuration file (or before building):
- File path after building: `/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
- File path before building: `/ws_livox/config`
- The host IP address needs to be set to `192.168.1.50`
  - The netmask needs to be `255.255.255.0`
  - The gateway needs to be `192.168.1.1`
- The sensor IP address needs to be set to `192.168.1.122`

### ROS2 Parameters
- The path to edit the ROS2 parameters is: `/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/` after building
- All internal parameters of Livox_ros_driver2 are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library (just for ROS) | 0       |

  **Note :**

  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **Note :**

  The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (only ROS can publish):

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

The description of the parameters is taken from the Livox ROS2 Driver GitHub

## Glim
To run Glim, first start the LiDAR topic and then run `ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)`