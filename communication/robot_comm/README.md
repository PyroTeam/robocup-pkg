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
Par exemple, on veut transmettre un message Landmarks :

* Ecrire le message protobuf Landmarks.proto correspondant au message ROS (dans le dossier proto_msg).
Ne pas oublier d'ajouter un code par défaut et un nom.

- Le message que l'on veut transmettre :

geometry_msgs/Pose2D landmarks

- Le message protobuf correspondant :

import "Pose2D.proto";
message Landmarks
{
	optional int32 code = 1 [default = 5];

	required string name = 2;

    required Pose2D landmarks = 3;
}

* Dans le CMakeList.txt : rajouter proto_msg/Landmarks.proto après PROTOBUF_GENERATE_CPP afin de le compiler automatiquement.

* Dans robotCommNode :
  - Rajouter le nouveau google protobuf message au msgCatalog : msgCatalog->add<Landmarks>();

  - Puis créer les topicToUdp et udpToTopic correspondant :
      UdpToTopicEntry<Landmarks, deplacement_msg::landmarks> udpToTopicLandmarks(udpPeer, "landmarks");
      TopicToUdpEntry<deplacement_msg::landmarks> topicToUdpLandmarks(udpPeer, "/landmarks");

  - Rajouter également le message au msgDispatcher :
      msgDispatcher->Add<Landmarks>(std::function<void(google::protobuf::Message&)>(boost::bind(&UdpToTopicEntry<Landmarks, deplacement_msg::landmarks>::execute, &udpToTopicLandmarks, _1)));

* Dans msgConvertUtils : Rajouter les fonctions rosToProtobuf et ProtobufToRos correspondant :

  - RosToProtobuf :
void rosToProtobuf(const boost::shared_ptr<const deplacement_msg::landmarks> &msg,
                   std::shared_ptr<google::protobuf::Message> &proto_msg, std::string topicName)
{
    std::shared_ptr<Landmarks> landmarks_proto(new Landmarks);

	landmarks_proto->set_name(topicName);
    geometry_msgs::Pose2D pose2D;
    Pose2D* pose = landmarks_proto->mutable_landmarks();
    pose->set_x(pose2D.x);
    pose->set_y(pose2D.y);
    pose->set_theta(pose2D.theta);
    proto_msg = landmarks_proto;
}

  - ProtobufToRos :
void ProtobufToRos(const Landmarks &proto_msg,
                   std::shared_ptr<deplacement_msg::landmarks> &msg)
{
    std::shared_ptr<deplacement_msg::landmarks> landmarks(new deplacement_msg::landmarks());

    Pose2D pose = proto_msg.landmarks();
    landmarks->landmarks.x = pose.x();
    landmarks->landmarks.y = pose.y();
    landmarks->landmarks.theta = pose.theta();

    msg = landmarks;
}

Ne pas oublier de rajouter les prototypes des fonctions dans msgConvertUtils.h.
