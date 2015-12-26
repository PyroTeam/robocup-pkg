#!/bin/bash

# Fresh install script
# ================================
# This script run a complete fresh install (not for robotino)

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"**************************" # Light Blue
echoC "lb"	"* freshInstall.sh script *"
echoC "lb"	"**************************"

$(dirname "$0")/checkRequest.sh
$(dirname "$0")/test.sh
$(dirname "$0")/installDependencies.sh
source ~/.bashrc
$(dirname "$0")/installCatkinWorkspace.sh
$(dirname "$0")/build.sh