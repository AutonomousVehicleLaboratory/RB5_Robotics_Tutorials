---
title: (1) Accessing the peripherals on Qualcomm Robotics RB5
date: 2022-02-15 17:12:21
categories:
  - 2 Accessing Devices
tags:
  - Camera
  - Sensor
---

A key sensor for many robotics applications is camera. It enables cool applications such as object detection, semantic segmentation and visual SLAM. There are two cameras on Qualcomm Robotics RB5.

This tutorial will tell you a few ways to access these cameras. Before we start, note that these cameras cannot be read from OpenCV directly but a tool called GStreamer can bridge the gap.

## OpenCV access through GStreamer and tcp

The easiest way to access camera is through a tcp port created by GStreamer. Then you can use OpenCV to read data from the tcp port.

On Qualcomm Robotics RB5, run the following command:

```
gst-launch-1.0 -e qtiqmmfsrc name=qmmf ! video/x-h264,format=NV12, width=1280, height=720,framerate=30/1 ! h264parse config-interval=1 ! mpegtsmux name=muxer ! queue ! tcpserversink port=8900 host=192.168.1.120
```

Note that you will need to change the host ip to your Qualcomm Robotics RB5's IP address. This can be done by running the following command.

```
sudo apt install net-tools # if you don't have ifconfig
ifconfig
```

The ip address of Qualcomm Robotics RB5 can be found after inet as something like 192.168.0.xxx.

Then you can access the camera with the help of the OpenCV library. A python example is given below

```
import cv2

cap = cv2.VideoCapture("tcp://192.168.1.120:8900") #rb5 ip & port (same from command)
while(True):
    ret, frame = cap.read()
    cv2.imwrite("captured_image_opencv.jpg",frame)
    # you can process image by using frame 
    break
cap.release()
```

Again, make sure you change the host ip to your Qualcomm Robotics RB5 IP.

## accessing camera using ROS or ROS2 packages

Another way is to use the ROS packages we provided both in ROS1 and ROS2.

ROS1: https://github.com/AutonomousVehicleLaboratory/rb5_ros
ROS2: https://github.com/AutonomousVehicleLaboratory/rb5_ros2

We provided launch file for both ROS and ROS2 packages so that it is easy to config the camera node.

### Access the Camera in ROS1

For ROS1 package, first clone it to your workspace.
```
# If you don't already have a works space, create one first.
mkdir -p rosws/src

# Go to the source folder
cd rosws/src

# Clone the repository
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros
```

Then, build your package.
```
# Return to the root folder of the workspace.
cd ..

# Include the ros tools, this assumes you have ROS1 melodic installed
source /opt/ros/melodic/setup.bash

# Build only this package
catkin_make --only-pkg-with-deps rb5_vision
```

Then you can run the package. For example, you can run the RGB camera by
```
# source the ros workspace
source devel/setup.bash

# start the program with a set of parameters in the launch file
roslaunch rb5_vision rb_camera_main_ocv.launch
```

This will publish images to the topic /camera_0.

start a new terminal and check with the following command.

```
source /opt/ros/melodic/setup.bash

rostopic list # list all the topics
rostopic hz /camera_0 # get the frequency of the topic
```

Finally, you can stop the process by pressing Ctrl + C in the terminal where it is running. Notice that it will take a few seconds for it to stop.


Similarly, you can run the tracking camera by
```
roslaunch rb5_vision rb_camera_side_ocv.launch
```

And this will publish images to the topic /camera_1.

### Access the camera in ROS2

For ROS2 package, first clone it to your workspace.
```
# If you don't already have a works space, create one first.
mkdir -p ros2ws/src

# Go to the source folder
cd ros2ws/src

# Clone the repository
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros2
```

Then, build your package.
```
# Return to the root folder.
cd ..

# Include the ros tools, this assumes you have ROS2 dashing installed
source /opt/ros/dashing/setup.bash

# Build only this package
colcon build --packages-select rb5_ros2_vision
```

Then you can run the package.
```
# Source the ROS2 workspace
source install/setup.bash

# Run the RGB camera
ros2 launch rb5_ros2_vision rb_camera_main_ocv_launch.py

# Or run the tracking camera
ros2 launch rb5_ros2_vision rb_camera_side_ocv_launch.py
```

Again, you can verify that the message is being published using the following command in another new terminal.
```
source /opt/ros/dashing/setup.bash
ros2 topic list # check if the topic is being pubilshed
ros2 topic hz /camera_0 # check the frequency of the RGB image message
```

Reference:
[1]: https://developer.qualcomm.com/comment/18637#comment-18637
