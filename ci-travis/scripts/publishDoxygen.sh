#!/bin/bash

# Publish doxygen script
# ======================
# This script generate and publish doxygen documentation on http://pyroteam.github.io/robocup-pkg/
# It will succeed only on ci-travis platform and for push on devel (not for pull-request nor for other branch push)

# Echo color function
source $(dirname "$0")/echoColor.sh

echo ""
echoC "lb"	"****************************" # Light Blue
echoC "lb"	"* publishDoxygen.sh script *"
echoC "lb"	"****************************"

# Exit on non ci-travis platform
if [ "$TRAVIS" != "true" ];then
	# RED
	echoC "r" "ERROR:"
	echoC "r" "This script must be only run on Ci-Travis platform"

	exit 0
fi

# Exit for other requests than push from /devel
if [ "$TRAVIS_PULL_REQUEST" != "false" -o "$TRAVIS_BRANCH" != "devel" ];then
	# Orange
	echoC "o" "WARNING: "
	echoC "o" "This script work only for /devel push"

	exit 0
fi

# Exit on first failure and verbose option
set -ev

# Settings
REPO_PATH=git@github.com:PyroTeam/robocup-pkg.git
HTML_PATH=~/catkin_ws/src/robocup-pkg/build/doc/html
COMMIT_USER="ValentinVERGEZ"
COMMIT_EMAIL="valentin.vergez@gmail.com"
# CHANGESET=$(git rev-parse --verify HEAD)
CHANGESET=$TRAVIS_COMMIT

# Get dependencies
sudo apt-get install --yes doxygen graphviz

# Get a clean version of the HTML documentation repo.
rm -rf ${HTML_PATH}
mkdir -p ${HTML_PATH}
git clone -b gh-pages "${REPO_PATH}" --single-branch ${HTML_PATH}

# Remove all the files through git to prevent stale files.
cd ${HTML_PATH}
git rm -rf .
cd ~/catkin_ws/src/robocup-pkg

# Generate the HTML documentation.
doxygen ci-travis/Doxyfile

# Create and commit the documentation repo.
cd ${HTML_PATH}
git add .
git config user.name "${COMMIT_USER}"
git config user.email "${COMMIT_EMAIL}"
git commit -m "Automated documentation build for changeset ${CHANGESET}."
git push origin gh-pages
cd -
