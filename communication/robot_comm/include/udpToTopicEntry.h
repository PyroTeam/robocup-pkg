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
#include <unordered_map>
#include "entryPoint.h"


template<class T, class P>
class UdpToTopicEntry : public EntryPoint
{
public:
    UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
    virtual ~UdpToTopicEntry(){}
    void execute(google::protobuf::Message &msg);

private:
    std::unordered_map<std::string, ros::Publisher> m_pubs;

};

template<class T, class P>
UdpToTopicEntry<T,P>::UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name) : EntryPoint(udpPeer, name)
{

}

template<class T, class P>
void UdpToTopicEntry<T,P>::execute(google::protobuf::Message &msg)
{
    T &m = dynamic_cast<T&>(msg);
    std::shared_ptr<P> rosMsg;

    ProtobufToRos(m, rosMsg);
    std::unordered_map<std::string, ros::Publisher>::const_iterator it = m_pubs.find(m.name());
    if(it != m_pubs.end())
    {
        it->second.publish(*rosMsg);
    }
    else
    {
        std::string name = "/test/";
        name.append(m.name());
        ros::Publisher pub = m_nh.advertise<P>(name, 1);
        pub.publish(*rosMsg);
        m_pubs[m.name()] = pub;
    }
}




#endif /* UDPTOTOPICENTRY_H_ */
