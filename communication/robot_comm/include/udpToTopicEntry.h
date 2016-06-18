/**
 * \file         udpToTopicEntry.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-16
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef UDPTOTOPICENTRY_H_
#define UDPTOTOPICENTRY_H_

#include <string>
#include <ros/ros.h>

#include "entryPoint.h"


template<class T, class P>
class UdpToTopicEntry : public EntryPoint
{
public:
 UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
 virtual ~UdpToTopicEntry(){}

private:
 ros::Publisher m_pub;
 bool execute(google::protobuf::Message &msg);


};

template<class T, class P>
UdpToTopicEntry<T,P>::UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name) : EntryPoint(name)
{
    m_pub = m_nh.advertise<P>("name", 1000);
}

template<class T, class P>
bool UdpToTopicEntry<T,P>::execute(google::protobuf::Message &msg)
{
    T &m = dynamic_cast<T&>(msg);
    P rosMsg;

    ProtobufToRos(m, rosMsg);

    m_pub.publish(rosMsg);
}




#endif /* UDPTOTOPICENTRY_H_ */
