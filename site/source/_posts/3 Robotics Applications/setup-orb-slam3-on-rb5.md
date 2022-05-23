---
title: (3) Setup ORB_SLAM3 on RB5
categories:
  - 3 Robotics Applications
tags:
  - SLAM Application
date: 2022-05-18 17:12:21
---

This tutorial will guide you towards running ORB_SLAM3 on RB5. ORB_SLAM3 is a popular software package that can perform visual SLAM and visual-inertial SLAM. The algorithm is fast and therefore suitable for the RB5 platform.

## Install ORB_SLAM3 Library

The code from ORB_SLAM3 original repository doesn't work right out of box on RB5. We created a version that we tested on RB5 and works well. You can download the code from https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5.

After downloading the code, follow the README.md to compile the ORB_SLAM3 library. 

## ROS wrapper for ORB_SLAM3 on RB5

We create wrapper for the ORB_SLAM3 api in both [ROS](https://github.com/AutonomousVehicleLaboratory/rb5_ros) and [ROS2](https://github.com/AutonomousVehicleLaboratory/rb5_ros2).

You will need to modify the CMakeLists.txt file in the ROS and ROS2 packages to give the correct ORB_SLAM3 library path so that these nodes can be built successfully.

When running the ROS node, you will need to access camera using the ROS package we mentioned in the basic tutorial [accessing cameras](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/15/2%20Accessing%20Devices/accessing-camera-on-rb5/).

For the ROS version, the wrapper allows you to run the Monocular version of ORB-SLAM3 on RB5. The text interface will work with wayland desktop.
{% image ORB-SLAM3-Gnome-Terminal.png ORB-SLAM3 running in a Gnome-Terminal %}

You can also launch RViz on the gnome desktop that you setup following the [tutorial](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/05/18/1%20Initial%20Set-up/visualization-tools/).

{% image ORB-SLAM3-RViz.png ORB-SLAM3 pose displayed in RViz %}

For the ROS2 version, the wrapper allows you to run the visual inertial SLAM with a single camera and the IMU on RB5. The IMU data can be accessed through a prebuilt ROS2 package that is tested on ROS2 dashing.