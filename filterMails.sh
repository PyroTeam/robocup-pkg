#!/bin/bash

## Only does it when stricly aware of what you are doing
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[${#BASH_SOURCE[@]}-1]}" )" && pwd )/"
if [ "$1" != "-f" ];then
	echo "mkdir ~/tmp"
	echo "cd ~/tmp"
	echo "git clone --bare https://github.com/PyroTeam/robocup-pkg.git"
	echo "cd robocup-pkg.git"

	echo ""
	echo "$SCRIPT_DIR/filterMails.sh"
	exit 666
fi

$SCRIPT_DIR/cleanScripts/cleanMails_Vincent2.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Vincent.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Sandra.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Thomas.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Romain.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Valentin.sh
rm -rf refs/original/
$SCRIPT_DIR/cleanScripts/cleanMails_Elise.sh
rm -rf refs/original/

echo ""
echo "git push --force --tags origin 'refs/heads/*'"
echo "cd .."
echo "rm -rf robocuk-pkg.git"
echo ""

exit 0
