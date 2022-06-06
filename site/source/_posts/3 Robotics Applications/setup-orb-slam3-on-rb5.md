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

After downloading the code, follow the README.md to compile the ORB_SLAM3 library. A simplified version are also given below.
```
# Install the required dependencies
git clone https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5
cd ORB_SLAM3_RB5
chmod +x build.sh
./build.sh
```

## ROS wrapper for ORB_SLAM3 on RB5

We create wrapper for the ORB_SLAM3 api in both [ROS](https://github.com/AutonomousVehicleLaboratory/rb5_ros) and [ROS2](https://github.com/AutonomousVehicleLaboratory/rb5_ros2). These wrappers handles communication and data conversion. The ROS version was tested on ROS melodic and the ROS2 version was tested on RO2 dashing.

The communication part includes receiving sensor messages published by other ROS node. The sensors include camera and IMU. If both of them are used, the wrapper also handles synchronization before passing them to the ORB_SLAM3 library. After the pose is given by the output from the ORB_SLAM3 library, we then convert it into transformation and publish to ROS TF.

We seperate the ORB_SLAM3 library and the ROS wrapper so that we don't need to include this library into the ROS workspace. You will need to modify the line 4 of the CMakeLists.txt file in the ROS and ROS2 packages to give the correct ORB_SLAM3 library path so that these nodes can be built successfully.

For the ROS1 
```
# If you don't already have a works space, create one first.
mkdir -p rosws/src

# Go to the source folder
cd rosws/src

# Clone the repository
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros
```

Before you build your package, set the path in the CMakeLists.txt file in the ORB_SLAM3_RB5 package from the repository that you just cloned.

```
set(ORB_SLAM3_SOURCE_DIR "path/to/ORB_SLAM3_RB5_LIB/")
```
Note that this path refers to the folder where the customized ORB_SLAM3 library from https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5 is cloned to.

Then, build your package.
```
# Return to the root folder of the workspace.
# Include the ros tools, this assumes you have ROS1 melodic installed
source /opt/ros/melodic/setup.bash

# Build only this package
catkin_make --only-pkg-with-deps ORB_SLAM3_RB5
```

To run the package, first start the roscore in a new terminal
```
source /opt/ros/melodic/setup.bash
roscore
```

Then in another terminal, go into the workspace folder
```
# source the ros workspace
source devel/setup.bash

# run the program
rosrun ORB_SLAM3_RB5 Mono /path/to/ORB_SLAM3_RB5_library/Vocabulary/ORBvoc.yaml /path/to/ORB_SLAM3_RB5_library/Examples/Monocular/Euroc.yaml
```
Notice that you should use a different yaml file to reflect the parameters of your camera.

When running the ROS node, you will need to access camera using the ROS package we mentioned in the basic tutorial [accessing cameras](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/15/2%20Accessing%20Devices/accessing-camera-on-rb5/).

For the ROS version, the wrapper allows you to run the Monocular version of ORB-SLAM3 on RB5. The text interface will work with wayland desktop.
{% image ORB-SLAM3-Gnome-Terminal.png ORB-SLAM3 running in a Gnome-Terminal %}

In the terminal, you can see the translation vector and rotation matrix being printed. A trajectory file named "KeyFrameTrajectory.txt" will be saved into the root of the workspace if the program is stoped by "Ctrl+C".

You can also launch RViz on the gnome desktop that you setup following the [tutorial](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/05/18/1%20Initial%20Set-up/visualization-tools/).

{% image ORB-SLAM3-RViz.png ORB-SLAM3 pose displayed in RViz %}

For the ROS2 version, the wrapper allows you to run the visual inertial SLAM with a single camera and the IMU on RB5. The IMU data can be accessed through a prebuilt ROS2 package that is tested on ROS2 dashing.

Similarly, first clone it to your workspace.
```
# If you don't already have a works space, create one first.
mkdir -p ros2ws/src

# Go to the source folder
cd ros2ws/src

# Clone the repository
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros2
```

Before you build your package, set the path in the CMakeLists.txt file in the ORB_SLAM3_RB5 package from the repository that you just cloned.

```
set(ORB_SLAM3_SOURCE_DIR "path/to/ORB_SLAM3_RB5_LIB/")
```

Note that this path refers to the folder where the customized ORB_SLAM3 library from https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5 is cloned to.

Then, build your package.
```
# Return to the root folder of the workspace.
# Include the ros tools, this assumes you have ROS2 dashing installed
source /opt/ros/dashing/setup.bash

# Build only this package
colcon build --packages-select orb_slam3_rb5_ros2
```

Then you can run the package.
```
# Source the ROS2 workspace
source install/setup.bash

# run the Monocular version
ros2 run orb_slam3_rb5_ros2 Mono /path/to/ORB_SLAM3_RB5_library/Vocabulary/ORBvoc.yaml /path/to/ORB_SLAM3_RB5_library/Examples/Monocular/Euroc.yaml

# run the Monocular-Inertial version
ros2 run orb_slam3_rb5_ros2 Mono_Inertial /path/to/ORB_SLAM3_RB5_library/Vocabulary/ORBvoc.yaml /path/to/ORB_SLAM3_RB5_library/Examples/Monocular-Inertial/Euroc.yaml
```

If You encounter error while loading shared libraries, add the library to path such as:
```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/path/to/ORB_SLAM3_RB5_LIB/lib:/path/to/Pangolin/build
```