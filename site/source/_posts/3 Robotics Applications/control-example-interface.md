---
title: (4) Control - An example interface using the Megabot Robot
tags:
  - control
  - usb
  - rc car
categories:
  - 3 Robotics Applications
date: 2022-05-22 17:12:21
---

As we described in our previous tutorial on [building and loading kernel modules on the RB5](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/05/18/2%20Accessing%20Devices/building-and-loading-kernel-modules/), the Ubuntu 18.04 installation that is part of the RB5 LU build includes minimal package support to reduce OS complexity. With this in mind, if we wish to install custom drivers, we need to perform the process described. Thankfully, if you have build and loaded the kernel modules described in our tutorial, you will be able to interface with a standard joystick controller over USB and serial over USB.

With the preliminaries completed, we will use the [Megabot Robot](https://store.makeblock.com/products/makeblock-mbot-mega-robot-kit) as an example to interface with an RB5. In this case, we will use the `megapi` Python module designed for the Megabot to communite with the robot. This can be readily installed using `pip install megapi`.

Here we define a set of primitive actions that can control this four-mechanum-wheel robot. The set of primitive control actions include move `left`,`right`,`forward`, `in reverse`, rotate `clockwise`, `counter-clockwise`, and `stop`. Yes, it come as a surprise to many but the wheels on the robot contain a number of different rollers that ultimately influence the kinematics of the robot and jointly provide very interesting properties such as rotating in place and moving sideways!

The inverse kinematics of the robot can be described below.

{% image inverse.png Inverse kinematics %}




The following code snippet implements the control conditions for each of the actions defined. 

```python
    def move(self, direction, speed):
        # port1: front right (wheel 1)
        # port2: rear left (wheel 0)
        # port9: rear right (wheel 3)
        # port10: front left (wheel 2)

        if direction == "left":
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)
            return
        elif direction == "right":
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
            return
        elif direction == "forward":
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, speed)


            return
        elif direction == "reverse":
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

            return
        elif direction == "ccwise":
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
        elif direction == "cwise":
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

        else:
            # fl wheel
            self.bot.motorRun(10, 0)

            # fr wheel
            self.bot.motorRun(1, 0)

            # rl wheel
            self.bot.motorRun(2, 0)

            # rr wheel
            self.bot.motorRun(9, 0)
            return

```

This script has been implemented using ROS1 and ROS2. While the robot can be controlled using the standard `joy` node using an USB jostick controller, it can additionally be controlled a standard ROS message for higher level planning strategies. The process of building and running the ROS nodes is outlined below.

## ROS1 

Create a workspace, clone the ROS1 implementation, and build the package. Make sure ROS is in your path, i.e. `source /opt/ros/melodic/setup.bash`. 

```
mkdir -p rb5_ws/src && cd rb5_ws/src
git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros.git
cd ..
catkin_make
source rb5_ws/devel/setup.bash 
```

Start the control node

```
ros run rb5_control rb5_mpi_control.py
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

Start the control node

```
ros2 run rb5_ros2_control rb5_mpi_control.py
```

## MegaBot

{% youtube 6Saki1CPfiA %}


### References

