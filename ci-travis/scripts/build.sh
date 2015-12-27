#!/bin/bash

# Build script
# ====================
# This script clone our ros repositories and run a catkin_make

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"*******************" # Light Blue
echoC "lb"	"* build.sh script *"
echoC "lb"	"*******************"

# Exit on first failure and verbose option
set -ev

# Configure environment
source ~/.bashrc

# Get robotino-pkg
git clone https://github.com/PyroTeam/robotino-pkg.git ~/catkin_ws/src/robotino-pkg

# Get robocup-pkg
if [ "$TRAVIS" = "true" ];then
	ln -s $TRAVIS_BUILD_DIR ~/catkin_ws/src/
else
	git clone https://github.com/PyroTeam/robocup-pkg.git ~/catkin_ws/src/robocup-pkg
fi

# Build
cd ~/catkin_ws
catkin_make