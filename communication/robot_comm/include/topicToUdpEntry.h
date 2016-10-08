/**
 * \file         topicToUdpEntry.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-11
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef TOPICTOUDPENTRY_H_
#define TOPICTOUDPENTRY_H_

#include <string>
#include <ros/ros.h>

#include "udpPeer.h"
#include "entryPoint.h"
#include "msgConvertUtils.h"

//a supprimer
#include <memory>
#include "comm_msg/activity.h"


template<class T>
class TopicToUdpEntry : public EntryPoint
{
public:
    TopicToUdpEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
    virtual ~TopicToUdpEntry(){}

private:
    ros::Subscriber m_sub;

    void Callback(const boost::shared_ptr<const T> &msg)
    {
        std::shared_ptr<google::protobuf::Message> proto_msg;

        //convert msg to google::protobuf::Message
        rosToProtobuf(msg, proto_msg, m_name);

        //send it via UdpPeer
        m_udpPeer->send(proto_msg);
        //std::cout << "Send udp msg" <<std::endl;
    }

};

template<class T>
TopicToUdpEntry<T>::TopicToUdpEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name) : EntryPoint(udpPeer, name)
{
    m_sub = m_nh.subscribe<T, TopicToUdpEntry>(m_name, 1000, &TopicToUdpEntry::Callback, this);
}




#endif /* TOPICTOUDPENTRY_H_ */
