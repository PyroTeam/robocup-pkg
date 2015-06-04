/**
 * \file         msgConvertUtils.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-11
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef MSGCONVERTUTILS_H_
#define MSGCONVERTUTILS_H_

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "Landmarks.pb.h"
#include "encryptUtils.h"

#include "comm_msg/activity.h"
#include "comm_msg/landmarks.h"

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>

void rosToProtobuf(const boost::shared_ptr<const comm_msg::activity> &msg,
                   std::shared_ptr<google::protobuf::Message> &proto_msg, std::string topicName);

void rosToProtobuf(const boost::shared_ptr<const comm_msg::landmarks> &msg,
                   std::shared_ptr<google::protobuf::Message> &proto_msg, std::string topicName);

void ProtobufToRos(const Activity &proto_msg,
                   std::shared_ptr<comm_msg::activity> &msg);

void ProtobufToRos(const Beacon &proto_msg,
                   std::shared_ptr<nav_msgs::Odometry> &msg);

void ProtobufToRos(const Landmarks &proto_msg, 
                   std::shared_ptr<comm_msg::landmarks> &msg);

#endif /* MSGCONVERTUTILS_H_ */
