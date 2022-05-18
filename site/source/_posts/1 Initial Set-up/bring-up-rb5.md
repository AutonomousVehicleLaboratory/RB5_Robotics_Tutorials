---
title: (1) Bring Up RB5
categories:
  - 1 Initial Set-up
date: 2022-02-13 17:12:21
tags:
---
We need to flash the Ubuntu 18.04 operating system to the RB5. The following steps comprise the process of flashing the OS to the board:

a) Install adb and fastboot by using the following command in Linux Terminal: 

  ```
  sudo apt-get install android-tools-adb android-tools-fastboot
  ```

b) Download the Qualcomm Robotics SDK Manager from [here](https://www.thundercomm.com/product/qualcomm-robotics-rb5-development-kit/#sdk-manager)

c) The download should be a Zip file that contains the SDK Manager installation package and a Readme file. Follow the instructions in the Readme file to install the prerequisites.

d) Install the SDK manager on the Linux workstation. Refer to the process given as step 2 in the following [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager)

e) Before running the SDK manager, if you are using a Linux workstation, run the following command in the Terminal: 
```
sudo systemctl stop ModemManager
```

f) Run the SDK manager. Follow step 3 in the following [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager)

g) Follow step 4 from the same [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager) to download resources and generate system image. This could take slightly more than 30 minutes. 

h) Choose LU or LE flash. The LU flash has been tried before and flash was a success. This [forum answer](https://developer.qualcomm.com/comment/18517) is a possible explanation for the difference between LU and LE flash.

i) Now, start the process of flashing the generated system images on the RB5 by following step 5 from the same [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager)

j) Follow steps 1-3 from this [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/flash-images) to continue and complete the flashing process successfully.

k) If flashing is successful, adb should be working. To check this, keep your RB5 connected to your workstation and open your workstation's terminal and type: 
```
adb shell
```
and you should see a device ID shown as an attached device.

l) If not, please power cycle the development kit. Since the system images are flashed, there is no need to press the F_DL key to force the device to enter the Emergency Download Mode.

Once the OS is flashed, the next step involves setting up WiFi and SSH connections.
To set up WiFi connectivity on RB5, follow steps 1-4 from this [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/set-up-network)

To access RB5 terminal, both adb shell or SSH can be used. To set up SSH connection:

a) Type the following commands in a new terminal:
```
    adb shell  
    sh-4.4#ifconfig 
```
The 'ifconfig' command gives you the IP address of your connection.
  Then use the following command:
```
   sh-4.4#ssh root@ <IP address>
```
 This will ask you for a password, which is 'oelinux123'
  
This will successfully complete the SSH connection, through which you can remotely access the RB5 terminal.
  
The next step involves connecting to a HDMI monitor. The following is the procedure:

  a) Refer to the 'Check HDMI' section in this [link](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/set-up-network)
  
  b) Instead of the 5 commands given in the link given in a), you could try just this single command after connecting the HDMI: 
  ```
  weston --connector=29
  ```
  c) If b) doesn't work, then use the following 5 commands everytime while connecting HDMI:
  ```
  c:\>adb shell
  sh-4.4# mkdir -p /usr/bin/weston_socket
  sh-4.4# export XDG_RUNTIME_DIR=/usr/bin/weston_socket
  sh-4.4# export LD_LIBRARY_PATH=/usr/lib:/usr/lib/aarch64-linux-gnu/
  sh-4.4# weston --tty=1 --connector=29 --idle-time=0
  ```
    


