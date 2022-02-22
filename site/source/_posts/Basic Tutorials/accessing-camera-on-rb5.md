---
title: (2) Accessing Camera on RB5
date: 2022-02-15 17:12:21
categories:
  - Basic Tutorials
tags:
  - Camera
  - Sensor
---

A few ways allow you to access the main camera from rb5. It cannot be read from OpenCV directly but a gstreamer can bridge the gap.

The easiest way to access camera is through a tcp port.

On RB5, run the following command:

```
gst-launch-1.0 -e qtiqmmfsrc name=qmmf ! video/x-h264,format=NV12, width=1280, height=720,framerate=30/1 ! h264parse config-interval=1 ! mpegtsmux name=muxer ! queue ! tcpserversink port=8900 host=192.168.1.120
```

Notice that you will need to change the host ip to your RB5 ip address.

Then you can access the camera from opencv by

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