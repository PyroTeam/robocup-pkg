#!/bin/bash

# Install catkin workspace script
# ===============================
# This script create and configure catkin_workspace

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"************************************" # Light Blue
echoC "lb"	"* installCatkinWorkspace.sh script *"
echoC "lb"	"************************************"

# Exit on first failure and verbose option
set -ev

# Configure environment
source ~/.bashrc

# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc