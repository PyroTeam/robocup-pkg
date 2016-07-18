# PREHENSION - HOW TO

## Installation des paquets nécessaires

* `sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial arduino`
* `sudo su`
* `source /home/<user>/catkin_ws/devel/setup.bash`
* copier le fichier firmware/arduinoGripper.ino dans le dossier sketchbook

## Génération de la librairie :

* `cd ~/sketchbook/libraries`
* `rm -rf ros_lib`
* `rosrun rosserial_arduino make_libraries.py /root/sketchbook/libraries/ gripper_msg`

## Compilation et upload du programme dans l'arduino.

## Utilisation
#### gripper (bas niveau)
* `roscore`
* `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0` (port série connecté à l'arduino)

*Le service "hardware/low_level/gripper_srv" et le topic "hardware/low_level/gripper_status" doivent apparaître*

appel du service :
* `rosservice call /hardware/low_level/gripper_srv "servo: false open: false"`

#### préhension (haut niveau)

* `rosrun gripper_node gripper_node`

*Le service "hardware/high_level/gripper_srv" doit apparaître*

appel du service préhension (take = 1, let = 2):
* `rosservice call /hardware/high_level/gripper_srv "cmd: 1"`
