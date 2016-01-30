Installation et des paquets nécessaires

* sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial arduino
* sudo su
* source /home/<user>/catkin_ws/devel/setup.bash
* copier le fichier firmware/arduinoGripper.ino dans le dossier sketchbook

Génération de ros_lib :

* cd sketchbook/libraries
* rm -rf ros_lib
* rosrun rosserial_arduino make_libraries.py /root/sketchbook/libraries/ gripper_msg

Compilation et upload du programme dans l'arduino.

Utilisation du node

* roscore
* rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

  (/dev/ttyACM0 étant le port série connecté à l'arduino)

Le service "gripper_srv" et le topic "gripper_status" doivent apparaître

Exemple d'appel du service pour tourner (servo du haut) et/ou pousser (servo du bas)

 rosservice call /gripper_srv "stateTurn: false statePush: false"
