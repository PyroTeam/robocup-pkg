/**
 * \file         entryPoint.h
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
#ifndef ENTRYPOINT_H_
#define ENTRYPOINT_H_

#include <string>
#include <ros/ros.h>
#include <google/protobuf/message.h>

class UdpPeer;

class EntryPoint
{
public:
    EntryPoint(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
    virtual ~EntryPoint();

    virtual bool execute(google::protobuf::Message &msg){}
protected:
    ros::NodeHandle m_nh;
    std::string m_name;
    std::shared_ptr<google::protobuf::Message> m_msg;
    std::shared_ptr<UdpPeer> m_udpPeer;
};

#endif /* ENTRYPOINT_H_ */
