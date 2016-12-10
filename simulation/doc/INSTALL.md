INSTALLATION - Environnement de simulation sur gazebo 
=====================================================

Cette documentation n'a pas vocation à expliquer en détails les procédures d'installation et de configuration de ROS.  
Quelques commandes d'installation de ROS seront tout de même données, mais ce document est rédigé pour un public ayant déjà effectué ou comprenant la procédure d'installation et de configuration de ROS.  
Si ce n'est pas le cas, suivez d'abord les instructions du [wiki ROS](http://wiki.ros.org/).  

**NotaBene** : La procédure d'installation expliquée ci-dessous est sujette à évolution, les commandes les plus à jour seront normalement toujours disponibles dans le script de "build-continu" suivant : [`installDependencies.sh`](/ci-travis/scripts/installDependencies.sh).

Packages ubuntu
---------------
Instructions valable sur Ubuntu 14.04 avec ROS JADE.  
Cette section permet d'installer une version de ROS et de Gazebo (le simulateur) compatibles entre elles et fonctionnelles sur l'environnement de simulation pour la RoboCup Logistic League.  

### Installation de ROS JADE ###
Si vous ne savez pas installer et configurer ROS, référez vous d'abord au [wiki ROS](http://wiki.ros.org/).  
Si vous avez déjà ces connaissances, voilà pour rappel la liste des commandes nécessaires à l'installation de ROS JADE :   
```Shell
# Configuraton des dépôts ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
# Installation de ROS full et autres packages utilisés par robocup-pkg
sudo apt-get install ros-jade-desktop-full ros-jade-ar-track-alvar ros-jade-ar-track-alvar-msgs ros-jade-joy
# Initialisation de rosdep
sudo rosdep init
rosdep update
# Ajout de ROS à l'environnement
echo "# ROS" >> ~/.bashrc
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installation de Gazebo 5 ###
```Shell
# Configuraton des dépôts gazebosim (osrfoundation)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
# Installation de Gazebo5
sudo apt-get install gazebo5
```

Gazebo-rcll
-----------
La simulation de l'environnement de la RoboCup Logistic League utilise le simulateur Gazebo avec des modèles et des plugins spécifiques. Ils sont disponibles dans le dépôt `gazebo-rcll` dont nous maintenons un fork.  

Procéduure d'installation des modèles et plugins :
```Shell
# Protobuf
sudo apt-get install libprotoc-dev libprotobuf-dev libprotobuf-c0-dev protobuf-c-compiler protobuf-compiler
# Récupération des sources
cd ~
git clone https://github.com/PyroTeam/gazebo-rcll.git
# Ajout de configuration spécifiques (chemin vers les modèles et plugins custom)
sudo sh -c 'echo "" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_RCLL=${HOME}/gazebo-rcll" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:\$GAZEBO_RCLL/worlds" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:\$GAZEBO_RCLL/plugins/lib/gazebo" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$GAZEBO_RCLL/models" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "# To include team specific models, also add for example this line:" >> /usr/share/gazebo/setup.sh'
sudo sh -c 'echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$GAZEBO_RCLL/models/pyro" >> /usr/share/gazebo/setup.sh'
# Ajout de la configuration à l'environnement
echo "# Gazebo (ROS)" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
# Compilation des plugins
cd ~/gazebo-rcll/plugins
make
```