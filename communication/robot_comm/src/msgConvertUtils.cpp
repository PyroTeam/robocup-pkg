/**
 * \file         msgConvertUtils.cpp
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



void ProtobufToRos(const Activity &proto_msg,
                   std::shared_ptr<comm_msg::activity> &msg)
{
    std::shared_ptr<comm_msg::activity> activity(new comm_msg::activity());

    msg->nb_robot = proto_msg.nb_robot();
    msg->state = int8(proto_msg.state());
    msg->machine_used = int8(proto_msg.machine_used());
    msg->nb_order = proto_msg.nb_order();

    msg = activity;
}
