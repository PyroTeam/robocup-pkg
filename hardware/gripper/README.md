TODO à détailler


- Installation (procédure pour hydro, on passera par cmake avec indigo) :
* installer rosserial_arduino,
* installer l'IDE arduino,
* copier le fichier firmware/arduinoGripper.ino dans le dossier sketchbook,
* génération de ros_lib :
 $ rosrun rosserial_arduino make_libraries.py /home/vincent/sketchbook/libraries/ gripper_msg

* compiler et uploader le programme dans l'arduino.

- Utilisation :
* lancer un roscore
* exécuter :
 $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
  (/dev/ttyACM0 étant le port série connecté à l'arduino)

* le service "gripper_srv" et le topic "gripper_status" doivent apparaître.
* exemple d'appel du service pour fermer la pince
 $ rosservice call /gripper_srv "state: false"
