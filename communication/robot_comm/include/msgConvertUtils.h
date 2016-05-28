/**
 * \file         msgConvertUtils.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-11
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef MSGCONVERTUTILS_H_
#define MSGCONVERTUTILS_H_

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "encryptUtils.h"

#include "comm_msg/activity.h"

void rosToProtobuf(const boost::shared_ptr<const comm_msg::activity> &msg,
                   std::shared_ptr<google::protobuf::Message> &proto_msg);

void ProtobufToRos(const std::shared_ptr<google::protobuf::Message> &proto_msg,
                   const boost::shared_ptr<const comm_msg::activity> &msg);

#endif /* MSGCONVERTUTILS_H_ */
