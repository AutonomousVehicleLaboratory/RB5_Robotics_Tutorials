---
title: (2) Accessing IMU on RB5
date: 2022-06-09 14:53:21
categories:
  - 2 Accessing Devices
tags:
  - IMU
  - Sensor
---

RB5 comes with an Inertial Measurement Unit (IMU). An IMU estimates the linear acceleration and angular velocity. These capabilities enable applications such as motion estimation and visual inertial SLAM, which are essential to many robotics tasks. The IMU can be accessed by a ROS2 node.

Assuming you have ROS2 dashing installed, then you can run the IMU node.
```
# source the ros dashing environment
source /opt/ros/dashing/setup.bash

# run the node
ros2 run imu-ros2node imu-ros2node
```

Check the published IMU messages in a new terminal.
```
# In a new terminal, source the environment.
source /opt/ros/dashing/setup.bash

ros2 topic list # list the topic
ros2 topic echo /imu # print the messages
```