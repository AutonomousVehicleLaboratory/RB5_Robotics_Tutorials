---
title: (1) ROS Installation
categories:
  - 3 Robotics Applications
tags:
  - ROS
  - install
date: 2022-05-21 17:12:21

---

This tutorial outlines the process of installing ROS which is short for Robot Operating System. While ROS is not an operating system in the traditional sense, this is an ecosystem that provides support for a wide variety of sensor drivers and software libraries to aid in the fast development of robotic applications. Whether it is to process camera data or to execute a motion plan to reach a target destination, ROS handles message passing between modules to enable communication across multiple software modules. 

In the following two subsections, we outline the installation process of two ROS versions, namely ROS1 (Melodic) and ROS2 (Dashing) that were specifically designed to run on native Ubuntu 18.04 systems. As the naming system suggests, ROS1 precedes ROS2; however, each can come with newer/older flavors for each (i.e. Kinetic, Melodic, Neotic are ROS1 versions and Dashing, Foxy, and Galactic are ROS2 flavors). While the operating system support varies across releases, the key differences between ROS1 and ROS2 involve package support and features. 

As far as benefits, a key benefit of using ROS2 involves security, stability, and its focus on making it compatible with industrial robotic applications that require reliability. However, some may find that open source packages that were previously availble in ROS1 are not entirely ported to ROS2. This is quickly changing but it is a tradeoff to consider during development.

In future tutorials, we will explore applications that utilize Melodic and Dashing as the basis for our applications since they are Ubuntu 18.04 specific. 

## Install ROS1 - Melodic

### Setup sources.list

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```



### Set up keys

```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### Install

```
sudo apt update
sudo apt install ros-melodic-desktop-full
```

### Source ROS environment

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Additional Dependencies for managing workspaces

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Install and initilize rosdep to help resolve package dependencies

```
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

### Verify installation by running RViz (visualization GUI)

```
rviz
```



[Reference](https://wiki.ros.org/melodic/Installation/Ubuntu)





## Install ROS2 - Dashing

### Install host operating system dependencies

```
sudo apt-get install usbutils git bc
sudo apt-get -y install locales
sudo apt-get update && sudo apt-get install curl gnupg2 lsb-release
```



### Setup sources.list

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```



### Set up keys

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

### Install

```
sudo apt-get install ros-dashing-desktop
```

### Source ROS environment

```
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Additional Dependencies for managing workspaces

```
sudo apt-get install python3-argcomplete
sudo apt-get install python3-colcon-common-extensions
```

### Verify installation by running Rviz2 (visualization GUI)

```
rviz2
```

[Reference](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/run-a-basic-ROS2-application)

