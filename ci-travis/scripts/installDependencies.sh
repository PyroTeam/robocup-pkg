#!/bin/bash

# Install dependencies script
# ===========================
# This script install all needed dependencies, from apt and from source

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"*********************************" # Light Blue
echoC "lb"	"* installDependencies.sh script *"
echoC "lb"	"*********************************"

# Verbose option
set -v

############################################
# Source lists
### ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116

### Gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

### Robotino API2
# sudo sh -c 'echo "deb http://packages.openrobotino.org/trusty trusty main" > /etc/apt/sources.list.d/openrobotino.list'
# Currently robotino-pkg are only compliant with robotino-api2 0.9, available on repository below
sudo sh -c 'echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" > /etc/apt/sources.list.d/openrobotino.list'

wget -qO - http://packages.openrobotino.org/keyFile | sudo apt-key add -

# Exit on first failure option
set -e

sudo apt-get update -qq

############################################
# Packages install
### ROS
sudo apt-get --yes --force-yes	install	ros-jade-desktop-full ros-jade-ar-track-alvar ros-jade-ar-track-alvar-msgs ros-jade-joy

### Gazebo
sudo apt-get --yes --force-yes	install gazebo5

### Robotino API2

if [ "$ROBOTINO_INSTALL" = "true" ];then
	sudo apt-get --yes --force-yes	install rec-rpc robotino-common robotino_daemons robotino-api2 robotino-examples robotino3-firmware
else
	sudo apt-get --yes --force-yes	install robotino-api2
fi


### Refbox
sudo apt-get --yes --force-yes	install libmodbus-dev protobuf-compiler libprotobuf-dev libprotoc-dev libboost-all-dev \
										libglibmm-2.4-dev libgtkmm-3.0-dev libncursesw5-dev libyaml-cpp-dev libavahi-client-dev\
										git libxt-dev libxaw7-dev libncurses5-dev autoconf autogen libtool libyaml-dev

### Protobuf
sudo apt-get --yes --force-yes	install libprotoc-dev libprotobuf-dev libprotobuf-c0-dev protobuf-c-compiler protobuf-compiler

############################################
# Compile dependencies
### Refbox
mkdir -p ~/tmp

#### Clips
cd ~/tmp
wget http://downloads.sourceforge.net/project/clipsmm/clips/6.30.0.20090722svn/clips-6.30.0.20090722svn.tar.bz2
tar xvjf clips-6.30.0.20090722svn.tar.bz2
rm ~/tmp/clips-6.30.0.20090722svn.tar.bz2
cd ~/tmp/clips-6.30.0.20090722svn
./configure && make
sudo make install
cd ~
rm -rf ~/tmp/clips-6.30.0.20090722svn

### Clipsmm
cd ~/tmp
wget http://downloads.sourceforge.net/project/clipsmm/clipsmm/clipsmm-0.3.4.tar.bz2
tar xvjf clipsmm-0.3.4.tar.bz2
rm ~/tmp/clipsmm-0.3.4.tar.bz2
cd ~/tmp/clipsmm-0.3.4
./autogen.sh && ./configure && make
sudo make install
cd ~
rm -rf ~/tmp/clipsmm-0.3.4

############################################
# Configure environment
### ROS
echo "# ROS" >> ~/.bashrc
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc

sudo rosdep init
rosdep update

### Gazebo-rcll
cd ~
git clone https://github.com/PyroTeam/gazebo-rcll.git

sudo sh -c 'echo "" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_RCLL=${HOME}/gazebo-rcll" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$GAZEBO_RCLL/worlds" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$GAZEBO_RCLL/plugins/lib/gazebo" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "# To include team specific models, also add for example this line:" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models/pyro" >> /usr/share/gazebo/setup.sh'

echo "# Gazebo (ROS)" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

cd ~/gazebo-rcll/plugins
make

### Refbox
mkdir ~/refbox
cd ~/refbox
git clone https://github.com/PyroTeam/llsf-refbox.git
cd ~/refbox/llsf-refbox
make
