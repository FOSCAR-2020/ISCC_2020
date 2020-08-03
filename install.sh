#!/bin/bash

## ros public key change
# sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sudo apt -y install python-pip
sudo apt -y install ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash

sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

git submodule init
git submodule update

sudo apt -y install ros-kinetic-openslam-gmapping
sudo apt -y install ros-kinetic-geodesy
sudo apt -y install ros-kinetic-geometry-msgs 
sudo apt -y install ros-kinetic-geographic-msgs
sudo apt -y install ros-kinetic-bfl

# For gps_goal package
sudo apt -y install ros-kinetic-move-base-msgs
sudo apt -y install ros-kinetic-click
sudo apt -y install ros-kinetic-swri-transform-util

# For move_base package
sudo apt -y install ros-kinetic-base-local-planner 
sudo apt -y install ros-kinetic-clear-costmap-recovery 
sudo apt -y install ros-kinetic-cmake-modules 
sudo apt -y install ros-kinetic-navfn
sudo apt -y install ros-kinetic-rotate-recovery 

sudo apt -y install libarmadillo-dev
sudo apt -y install python-visual

sudo pip install geographiclib

cd ~/ISCC_2019/ && catkin_make

echo "source ~/ISCC_2019/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo su
echo -e SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyGPS1"\\nSUBSYSTEM=="tty", ATTRS{serial}=="0001", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyIMU" > /etc/udev/rules.d/99-usb-serial.rules
exit
