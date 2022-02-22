---
title: (3) Setup Gnome Desktop on RB5
date: 2022-02-13 17:12:21
categories:
  - Basic Tutorials
tags:
---

Getting a gnome desktop is desirable for its more familiar to most of us compared to the default wayland desktop. 

To get the gnome desktop, you will need to unminimize the system first. The current system is a minimal ubuntu 18.04 server and you can unminimize it to add more tools.

Then you can start install the gnome desktop by the following commands:

```
apt install gdm3 tasksel
tasksel install ubuntu-desktop
```

When this is done, you need to run the following command everytime you want to us the gnome desktop.

```
service gdm3 start
```

Notice that in order to successfully login to the gnome desktop, make use you choose the wayland for ubuntu in the login page, otherwise your username and password might not work.

Notice that even we have this gnome desktop, many gui applications with x11 support might still not work. However, the gnome-terminal and RViz will work properly.

Also, we do not recommand use this desktop because it takes a lot of computation (300-400% cpu!!!). It should only be used for visualization and debugging purpose for a short period of time.

Reference:
[1] https://linuxconfig.org/how-to-install-gnome-on-ubuntu-18-04-bionic-beaver-linux