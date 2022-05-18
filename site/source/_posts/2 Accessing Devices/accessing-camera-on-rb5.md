---
title: (1) Accessing Camera on RB5
date: 2022-02-15 17:12:21
categories:
  - 2 Accessing Devices
tags:
  - Camera
  - Sensor
---

A key sensor for many robotics applications is camera. It enables cool applications such as object detection, semantic segmentation and visual SLAM. There are two cameras on RB5.

This tutorial will tell you a few ways to access these cameras. Before we start, note that these cameras cannot be read from OpenCV directly but a tool called GStreamer can bridge the gap.

The easiest way to access camera is through a tcp port created by GStreamer. Then you can use OpenCV to read data from the tcp port.

On RB5, run the following command:

```
gst-launch-1.0 -e qtiqmmfsrc name=qmmf ! video/x-h264,format=NV12, width=1280, height=720,framerate=30/1 ! h264parse config-interval=1 ! mpegtsmux name=muxer ! queue ! tcpserversink port=8900 host=192.168.1.120
```

Note that you will need to change the host ip to your RB5 ip address. This can be done by running the following command.

```
sudo apt install net-tools # if you don't have ifconfig
ifconfig
```

The ip address of RB5 can be found after inet as something like 192.168.0.xxx.

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

Again, make sure you change the host ip to your RB5 ip.

Another way is to use the ROS package we provided both in ROS1 and ROS2.

ROS1: https://github.com/AutonomousVehicleLaboratory/rb5_ros/tree/main/rb5_vision
ROS2: https://github.com/AutonomousVehicleLaboratory/rb5_ros2/tree/main/rb5_ros2_vision

Reference:
[1]: https://developer.qualcomm.com/comment/18637#comment-18637