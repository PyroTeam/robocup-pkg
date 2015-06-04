robot_comm
==============
Package de communication entre les robotinos

Description
-----------
Permet l'utilisation des trois robotinos au cours d'une partie grâce à la communication entre ces derniers qui est réalisée dans ce package.

Noeuds
------
* robotCommNode : programme principal réalisant la communication entre les robots

Launchfiles
-----------
* `robotComm.launch` : lance le noeud robotCommNode
* `robotCommddd.launch` : lance le noeud robotCommNode en mode debug
* `robotCommValgrind.launch` : lance le noeud robotCommNode en vérifiant qu'il n'y ait pas de fuites mémoire

Scripts
-------

Avancement
----------
* [x] Rédiger un README
* [x] Réussir à établir une communication
  - [x] Topics
  - [ ] Services

Ajouter un nouveau topic
------------------------
* Ecrire le message protobuf correspondant au message ROS (dans le dossier proto_msg), ne pas oublier d'ajouter un code par défaut et un nom
* Dans le CMakeList.txt : rajouter le message protobuf après PROTOBUF_GENERATE_CPP afin de le compiler automatiquement
* Dans robotCommNode :
  - Rajouter le nouveau google protobuf message au msgCatalog
  - Puis créer les topicToUdp et udpToTopic correspondant
  - Rajouter également le message au msgDispatcher
* Dans msgConvertUtils : Rajouter les fonctions rosToProtobuf et ProtobufToRos correspondant
