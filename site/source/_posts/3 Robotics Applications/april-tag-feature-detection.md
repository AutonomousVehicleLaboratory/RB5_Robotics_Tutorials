---
title: (2) April Tag Feature Detection
tags:
  - Feature Detection
categories:
  - 3 Robotics Applications
date: 2022-02-13 17:12:21
---

An important task in robotics is to uniquely identify features and landmarks over time as a robot navigates through the environment. The process can help estimate the robotics position (localization) to ultimately be able to perform robust path planning and navigation. In this tutorial, we will explore a popular open-source library for AprilTag detection and 3D pose estimation from a single monorcular camera. 

AprilTags (shown in the figure below) are a form of fiducial markers that are designed with predefined sizes and patterns. The benefit of knowing their size is pose estimation given that a perspective mapping between two planes can be described by a Homography $\mathbf{H}$ matrix. In this case, $\mathbf{H}$ can describe a mapping between the camera frame and the marker since both can be assumed to be flat surfaces. This makes AprilTags an easy way of performing 3D pose estimation of objects with a single monocular camera by simply placing an AprilTag on the object. In addition, the unique pattern that defines each marker can help differentiate multiple markers that may be observable at a particular instance and by leveraging the concept of Hamming distance and dictionaries, error detection and correction can be performed in the event that part of a marker is occluded.

{% image apriltags.png Multiple AprilTag fiducial markers detected within an image with IDs 7 and 237. For marker 7, an error is detected and corrected. [1] %}
 



## The AprilTag3 Library

Using the AprilTag library is actually quite simple and can be installed from [source](https://github.com/AprilRobotics/apriltag). This includes compatible versions for C++ but also Python3. While our implementation will consist of C++, the Python version is even easier to get up and running. To install, simply run `pip install apriltag`.

For convenience, we have provided two implementations for AprilTag detection in ROS1 and ROS2. Given that both are C++ implementations, the logic remains the same. First, we convert our image to grayscale and populate a `image_u8_t` struct using the OpenCV `cv::Mat` image format. 

```c++
cv::Mat image_gray; 
cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

image_u8_t im = { .width  = image_gray.cols,
.height = image_gray.rows,
.stride = image_gray.cols, 
.buf    = image_gray.data 
};
```

 Once the image has been converted to grayscale, we can instantiate an `apriltag_detection_t` instance and call `apriltag_detector_detect()`. This is the primary function incharge of performing marker detection on camera data. It is worth noting that this detection object can be reused so memory can be allocated in the class constructor of your implementation.

```c++
apriltag_detector_t *a_detector = apriltag_detector_create();
zarray_t * detections = apriltag_detector_detect(a_detector, &im);
```

As it can be seen, `apriltag_detector_detect()` returns an array of type `zarray_t` with the list of detections concatenated. In the following code block we extract individual detections into instances of type `apriltag_detection_t` and perform a perspective mapping using the homography matrix calculated. This is handled by `estimate_tag_pose()`. However, two important considerations include *i)* that we know the size of the markers in advance, and *ii)* we understand the intrinsic parameters of the camera that include image center and focal length. These are attributes that are part of the first argument of type `apriltag_detection_info_t` that is passed to `estimate_tag_pose()`. 
```c++
apriltag_detection_t *det;
vector<apriltag_pose_t> poses;
vector<int> ids;

for (int i=0; i<zarray_size(detections); i++){

  zarray_get(detections, i, &det);
  info.det = det;
  apriltag_pose_t pose;

  // estimate SE(3) pose 
  estimate_tag_pose(&info, &pose);
  poses.push_back(pose);
  ids.push_back(det->id);
}
```
The marker size used in our implementation corresponds to $0.162m$ (depending on the print AprilTag marker size) and the camera parameters are estimated for the wide angle lens of the Qualcomm Robotics RB5 using the [OpenCV calibration tool](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) or its [ROS version](http://wiki.ros.org/camera_calibration) and a standard checkerboard target. Notice that after you calibrate your camera and undistort the image, the undistort image will have a new camera matrix. We set the info for AprilTag to perform accurate pose estimation after getting the new camera matrix.
```c++
cv::Mat rectify(const cv::Mat image){
  cv::Mat image_rect = image.clone();
  // get new camera matrix after undistort
  // alpha is set to 0.0 so all pixels are valid
  const cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, d, image.size(), 0.0);
  cv::undistort(image, image_rect, K, d, new_K);

  // set info for pose estimation using new camera matrix
  det.setInfo(tagSize, new_K.at<double>(0,0), new_K.at<double>(1,1), new_K.at<double>(0,2), new_K.at<double>(1,2));

  return image_rect;
}

void AprilDetection::setInfo(double tagSize, double fx, double fy, double cx, double cy){
  info.tagsize = tagSize;
  info.fx = fx;
  info.fy = fy;
  info.cx = cx;
  info.cy = cy;
}
```
The components described above have been wrapped into ROS1 and ROS2 implementations and be evaluated using the steps below. You will need to set the following parameters in the node.
```c++
// TODO: Replace these parameters using your calibration results
double distortion_coeff[] =
        {0.008549, -0.016273, -0.002954, -0.003708, 0.000000};
double intrinsics[] = {683.558755,    0.     ,  961.607445,
                       0.     ,  680.809134,  547.701668,
                       0.     ,    0.     ,    1.};

const cv::Mat d(cv::Size(1, 5), CV_64FC1, distortion_coeff);
const cv::Mat K(cv::Size(3, 3), CV_64FC1, intrinsics);

// TODO: Set tagSize for pose estimation, assuming same tag size.
// details from: https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
const double tagSize = 0.162; // in meters
```

## ROS1 

Create a workspace, clone the ROS1 implementation, and build the package. Make sure ROS is in your path, i.e. `source /opt/ros/melodic/setup.bash`. 

```
# install dependency
apt install ros-melodic-apriltag

mkdir -p rb5_ws/src && cd rb5_ws/src
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros.git
cd ..

# build all packages in the workspace
catkin_make
source devel/setup.bash

# if the build all packages failed, try build only
catkin_make --only-pkg-with-deps rb5_vision
catkin_make --only-pkg-with-deps april_detection
```

Start the camera node

```
roslaunch rb5_vision rb_camera_main_ocv.launch
```

Start the AprilTag detection node

```
rosrun april_detection april_detection_node
```



## ROS2

Create a workspace, clone the ROS2 implementation, and build the package. Make sure ROS is in your path, i.e. `source /opt/ros/dashing/setup.bash`. 

```
mkdir -p rb5_ws/src && cd rb5_ws/src
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros2.git
cd ..
colcon build
source rb5_ws/install/setup.bash 
```

Start the camera node

```
ros2 launch rb5_ros2_vision rb_camera_main_ocv_launch.py
```

Start the AprilTag detection node

```
ros2 run ros2_april_detection april_detection_node
```



## Visualizing the markers and poses

For convenience, our ROS implementations publish messages of type `geometry_msgs::PoseArray` ([ROS1](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html)) and `geometry_msgs::msg::PoseStamped` ([ROS2](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html)). These messages are timestamped and include a unique ID that corresponds to the marker ID as part of the message header. Nonetheless, the main component of each is message is the marker's pose in 3D which is represented as a position (a point) and orientation (here represented as a Quaternion). While we won't go into the math component on how Quaternions are utilized to represent orientations in 3D space, ROS has an good tools for handling transformation and can handle transformations with ease. Below is a video of each marker being visualized using the 3D visualization tool for ROS called RViz. The video demos AprilTag3 detection and real-time 3D pose estimation running onboard of Qualcomm RB5.

{% youtube qRoW6ljBfFo %}


### References

[1] [AprilTag: A robust and flexible visual fiducial system](https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf)
