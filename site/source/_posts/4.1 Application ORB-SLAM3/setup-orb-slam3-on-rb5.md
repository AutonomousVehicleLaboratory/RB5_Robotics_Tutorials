---
title: (3) Setup ORB_SLAM3 on RB5
categories:
  - 4.1 Application ORB-SLAM3
tags:
  - SLAM Application
date: 2022-02-13 17:12:21
---

This tutorial will guide you towards running ORB_SLAM3 on RB5.

The code from ORB_SLAM3 original repository doesn't work right out of box on RB5. We created a version that we tested on RB5 and works well. You can download the code from https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5.

After downloading the code, follow the README.md to compile the ORB_SLAM3 library and ROS nodes. When running the ROS node, you will need to access camera using the ROS package we mentioned in the basic tutorial accessing cameras https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/15/1%20Basic%20Tutorials/accessing-camera-on-rb5/.

The current version allows you to run the Monocular version of ORB-SLAM3 on RB5. The text interface will work with wayland desktop.
{% image ORB-SLAM3-Gnome-Terminal.png ORB-SLAM3 running in a Gnome-Terminal %}

You can also launch RViz on the gnome desktop that you setup following the tutorial: https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/13/1%20Basic%20Tutorials/setup-gnome-desktop-on-rb5/.

{% image ORB-SLAM3-RViz.png ORB-SLAM3 pose displayed in RViz %}