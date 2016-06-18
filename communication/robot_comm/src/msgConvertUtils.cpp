/**
 * \file         msgConvertUtils.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-11
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "msgConvertUtils.h"

void rosToProtobuf(const boost::shared_ptr<const comm_msg::activity> &msg,
                   std::shared_ptr<google::protobuf::Message> &proto_msg)
{
    std::shared_ptr<Activity> activity_proto(new Activity);

    activity_proto->set_nb_robot(msg->nb_robot);
    activity_proto->set_state(Activity::STATE_ROBOT(msg->state));
    activity_proto->set_machine_used(Activity::MACHINE_TYPE(msg->machine_used));
    activity_proto->set_nb_order(msg->nb_order);

    proto_msg = activity_proto;
}



void ProtobufToRos(const std::shared_ptr<google::protobuf::Message> &proto_msg,
                   const boost::shared_ptr<comm_msg::activity> &msg)
{
    std::shared_ptr<comm_msg::activity> activity(new comm_msg::activity());
}
