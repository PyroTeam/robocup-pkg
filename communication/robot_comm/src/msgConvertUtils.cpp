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
    //TODO conversion
    activity_proto->set_state(Activity::IN_PROGRESS);
    activity_proto->set_machine_used(Activity::DS);
    activity_proto->set_nb_order(msg->nb_order);

    proto_msg = activity_proto;
}
