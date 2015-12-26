#!/bin/bash

# Fresh install on robotino script
# ================================
# This script run a complete fresh install for robotino (using freshInstall.sh script)

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"***********************************" # Light Blue
echoC "lb"	"* freshInstallRobotoino.sh script *"
echoC "lb"	"***********************************"

export ROBOTINO_INSTALL=true
source $(dirname "$0")/freshInstall.sh