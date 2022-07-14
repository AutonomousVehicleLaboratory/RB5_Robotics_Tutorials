---
title: (3) Building and Loading Kernel Modules
date: 2022-05-18 17:12:21
categories:
  - 2 Accessing Devices
tags:
  - System
---

The LU build outlined during in the [bring-up process](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/02/13/1%20Initial%20Set-up/bring-up-rb5/) comprises of a minimal Ubuntu 18.04 installation. For this reason, various device kernel modules need to be build from source and loaded. In this tutorial, we document the process of building and loading the kernel modules for a USB joystick (**joydev**) and USB over serial (**ch341**). The source code associated with these modules is open-source and available as part of the [linux kernel](https://github.com/torvalds/linux). We suggest use the USB-C cable to connect your RB5 to the computer so that you can copy large block of code over. Make sure you check the correctness of the format.

### joydev

The kernel version utilized for this tutorial corresponds to `4.19.125`. If a different version is being used, you can find the version that matches your kernel by utilizing `uname -r`.

Extract the source code associated with this module, copy it to a temporary directory, for example, we use directory `joydev`.

```
wget https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.19.125.tar.gz
tar xvzf linux-4.19.125.tar.gz

mkdir joydev
cp -r linux-4.19.125/drivers/input/* joydev/ && cd joydev/
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/input/
```

The following code need to be appended to the end of the Makefile that was copied to the temporary directory `joydev`.

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

To avoid loading the module every time, create a script outside of the directory.

```
cd ..
vim joydev.sh
```

then copy the following script into the file.

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
  cp ./joydev/joydev.ko /lib/modules/$(uname -r)/kernel/drivers/input/
  depmod -a
  echo "JOYDEV loaded"
fi
```

save the file and execute it with
```
bash joydev.sh
```

The joydev module will be copied into the kernel directory and dynamically loaded when a joystick device is found.

### ch341

Extract the source code associated with this module to a temporary directory, in this case `ch341`

```
# Skip these two steps if you already did it
wget https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.19.125.tar.gz
tar xvzf linux-4.19.125.tar.gz 

mkdir ch341
cp -r linux-4.19.125/drivers/usb/serial/* ch341 && cd ch341
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
```

The following code need to be append to the end of the Makefile that was copied to `ch341`.

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

To avoid loading the module every time, create a script outside of the directory.

```
cd ..
vim ch341.sh
```

then copy the following script into the file.
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
  cp ./ch341/ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
  depmod -a
  echo "CH341 loaded"
fi
```

save the file and execute it with
```
bash ch341.sh
```


The Makefiles and kernel modules can be found on [Github](https://github.com/AutonomousVehicleLaboratory/rb5_lib).