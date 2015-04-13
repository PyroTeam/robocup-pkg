/**
 * \file         inboundPoint.h
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
#ifndef INBOUNDPOINT_H_
#define INBOUNDPOINT_H_

#include <string>
#include <ros/ros.h>

#include "entryPoint.h"
#include "msgConvertUtils.h"

//a supprimer
#include <memory>
#include "comm_msg/activity.h"


template<class T>
class InboundPoint : public EntryPoint
{
public:
    InboundPoint(const std::string &name);
    virtual ~InboundPoint(){}

private:
    ros::Subscriber m_sub;

    void Callback(const boost::shared_ptr<const T> &msg)
    {
        std::shared_ptr<google::protobuf::Message> proto_msg;
        //convert msg to google::protobuf::Message
        rosToProtobuf(msg, proto_msg);
        //test à supprimer
        if(std::shared_ptr<Activity> a = std::dynamic_pointer_cast<Activity>(proto_msg))
        {
            std::cout << "nb robot = " << a->nb_robot() << std::endl;
        }

        //send it via UdpPeer
        //m_udpPeer.send(proto_msg);
        std::cout << "Receive msg" <<std::endl;
    }

};

template<class T>
InboundPoint<T>::InboundPoint(const std::string &name) : EntryPoint(name)
{
    m_sub = m_nh.subscribe<T, InboundPoint>(m_name, 1000, &InboundPoint::Callback, this);
}




#endif /* INBOUNDPOINT_H_ */
