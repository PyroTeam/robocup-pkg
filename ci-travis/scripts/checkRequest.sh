#!/bin/bash

# Check request script
# ====================
# This script show some environment configurations and display an understanble description of requested build

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"**************************" # Light Blue
echoC "lb"	"* checkRequest.sh script *"
echoC "lb"	"**************************"

#################################################
# Display a var if setted from varname
# example :
# 'dispVar varA' will do : 'echo "varA : $varA"'
################################################
function dispVar
{
	if [ $# -eq 1 ]; then
		eval "test -v $1"
		if [ $? -eq 0 ]; then
			eval "VAR=\$$1"
			echo "$1 : $VAR"
		else
			echo "$1 -not set-"
		fi
	else
		echoC "r" "ERROR : dispVar function expects one argument"
	fi
}


########################
# Dump expected env vars
########################
echo ""
echoC "b" "Dump some expected environment variables" # Blue
echoC "b" "========================================"

## Expected from Ci−Travis
dispVar CI
dispVar TRAVIS
dispVar CONTINUOUS_INTEGRATION
dispVar TRAVIS_BRANCH
dispVar TRAVIS_BUILD_DIR
dispVar TRAVIS_COMMIT
dispVar TRAVIS_COMMIT_RANGE
dispVar TRAVIS_PULL_REQUEST
dispVar TRAVIS_TAG

## Excpeted on normal build
dispVar UPDATE_INSTALL
dispVar ROBOTINO_INSTALL


#############################
# Display build configuration
#############################
echo ""
echoC "b" "Requested build" # Blue
echoC "b" "==============="

if [ "$CI" = "true" -o "$CONTINUOUS_INTEGRATION" = "true" ]; then
	echo "Continuous integration"
	if [ "$TRAVIS" = "true" ];then
		echo "On Ci-Travis build platform"
	fi
else
	echo "Normal build"
	if [ "$UPDATE_INSTALL" != "true" ];then
		echo "On a fresh install"
	else
		echo "On an update install"
	fi

	if [ "$ROBOTINO_INSTALL" = "true" ];then
		echo "Specifically on a Robotino"
	fi
fi