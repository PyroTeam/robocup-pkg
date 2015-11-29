#!/bin/bash

## Only does it when stricly aware of what you are doing
if [ "$1" != "-f" ];then
	echo "mkdir ~/tmp"
	echo "cd ~/tmp"
	echo "git clone --bare https://github.com/PyroTeam/robocup-pkg.git"
	echo "cd robocup-pkg.git"

	exit 666
fi

./cleanScripts/cleanMails_Vincent2.sh
rm -rf refs/original/
cleanScripts/cleanMails_Vincent.sh
rm -rf refs/original/
./cleanScripts/cleanMails_Sandra.sh
rm -rf refs/original/
./cleanScripts/cleanMails_Thomas.sh
rm -rf refs/original/
./cleanScripts/cleanMails_Romain.sh
rm -rf refs/original/
./cleanScripts/cleanMails_Valentin.sh
rm -rf refs/original/
./cleanScripts/cleanMails_Elise.sh
rm -rf refs/original/

echo ""
echo "git push --force --tags origin 'refs/heads/*'"
echo "cd .."
echo "rm -rf robocuk-pkg.git"
echo ""

exit 0
