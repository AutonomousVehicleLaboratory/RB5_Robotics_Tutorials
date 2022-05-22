---
title: (2) Building and Loading Kernel Modules
date: 2022-05-18 17:12:21
categories:
  - 2 Accessing Devices
tags:
  - System
---

The LU build outlined during in the [bring-up process](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/13/1%20Initial%20Set-up/bring-up-rb5/) comprises of a minimal Ubuntu 18.04 installation. For this reason, various device kernel modules need to be build from source and loaded. In this tutorial, we document the process of building and loading the kernel modules for a USB joystick (**joydev**) and USB over serial (**ch341**). The source code associated with these modules is open-source and available as part of the [linux kernel](https://github.com/torvalds/linux).

### joydev

The kernel version utilized for this tutorial corresponds to `4.19.125`. If a different version is being used, you can find the version that matches your kernel by utilizing `uname -r`.

Extract the source code associated with this module

```
wget https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.19.125.tar.gz
tar xvzf linux-4.19.125.tar.gz 
cp -r linux-4.19.125/drivers/input /temp/dir && cd /temp/dir/input
```

The following changes need to be made to the Makefile that was copied to `/temp/dir/input`.

```makefile
KVERS = $(shell uname -r)

# kernel modules
obj-m := joydev.o
#
EXTRA_CFLAGS=-g -O0 -Wno-vla -Wframe-larger-than=4496

build: kernel_modules

kernel_modules:
	make -C /usr/src/header M=$(CURDIR) modules

clean:
	make -C /usr/src/header M=$(CURDIR) clean
```

Build and Load kernel module

```
make
insmod joydev.ko
```

To avoid loading the module every time, the following script can be utlized.

```shell
#!/bin/bash

KERNEL_VERSION=$(uname -r)
MODINFO=$(modinfo ./joydev/joydev.ko | grep vermagic)
MODULE_VERSION=$(echo $MODINFO | cut -d " " -f 2) 


if [ $KERNEL_VERSION != $MODULE_VERSION ]
then
  echo "Versions incompatible"
  echo ".ko file compiled with " $MODULE_VERSION
  echo "System kernel is " $KERNEL_VERSION
else
  cp ./joydev/joydev.ko /lib/modules/$(uname -r)/kernel/drivers/input
  depmod -a
  echo "JOYDEV loaded"
fi
```



### ch341

Extract the source code associated with this module

```
wget https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.19.125.tar.gz
tar xvzf linux-4.19.125.tar.gz 
cp linux-4.19.125/drivers/usb/serial && cd /temp/dir/input
```

The following changes need to be made to the Makefile that was copied to `/temp/dir/input`.

```makefile
KVERS = $(shell uname -r)

# kernel modules
obj-m := ch341.o

EXTRA_CFLAGS=-g -O0 -Wno-vla

build: kernel_modules

kernel_modules:
	make -C /usr/src/header M=$(CURDIR) modules

clean:
	make -C /usr/src/header M=$(CURDIR) clean
```

Build and Load kernel module

```
make
insmod ch341.ko
```

To avoid loading the module every time, the following script can be utlized.

```shell
#!/bin/bash

KERNEL_VERSION=$(uname -r)
MODINFO=$(modinfo ./ch341/ch341.ko | grep vermagic)
MODULE_VERSION=$(echo $MODINFO | cut -d " " -f 2) 


if [ $KERNEL_VERSION != $MODULE_VERSION ]
then
  echo "Versions incompatible"
  echo ".ko file compiled with " $MODULE_VERSION
  echo "System kernel is " $KERNEL_VERSION
else
  cp ./ch341/ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial
  depmod -a
  echo "CH341 loaded"
fi
```



The Makefiles and kernel modules can be found on [Github](https://github.com/AutonomousVehicleLaboratory/rb5_lib).