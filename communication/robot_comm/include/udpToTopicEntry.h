/**
 * \file         udpToTopicEntry.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-16
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef UDPTOTOPICENTRY_H_
#define UDPTOTOPICENTRY_H_

#include <string>
#include <ros/ros.h>

#include "entryPoint.h"


template<class T, class P>
class udpToTopicEntry : public EntryPoint
{
public:
 udpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
 virtual ~udpToTopicEntry(){}

private:
 ros::Publisher m_pub;


};

template<class T, class P>
TopicToUdpEntry<T>::TopicToUdpEntry(const std::string &name) : EntryPoint(name)
{
    m_pub = m_nh.advertise<P>("name", 1000);
}

template<class T, class P>
bool udpToTopicEntry<T>::execute(google::protobuf::Message &msg)
{
    T &m = dynamic_cast<T&>(msg);
    P rosMsg;

    ProtobufToRos(m, rosMsg);

    m_nh.publish(rosMsg);
}




#endif /* UDPTOTOPICENTRY_H_ */
