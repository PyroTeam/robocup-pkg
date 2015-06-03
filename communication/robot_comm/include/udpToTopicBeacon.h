/**
 * \file         udpToTopicEntryBeacon.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-06-02
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef UDPTOTOPICBEACON_H
#define UDPTOTOPICBEACON_H

#include <string>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>
#include "udpToTopicEntry.h"

#include <nav_msgs/Odometry.h>
#include "comm_msg/active_robot.h"
#include "comm_msg/active_robots.h"

template <>
class UdpToTopicEntry<Beacon, nav_msgs::Odometry> : public EntryPoint
{
public:
    UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name);
    virtual ~UdpToTopicEntry(){}
    void execute(google::protobuf::Message &msg);
    void getActiveRobots(std::vector<comm_msg::active_robot>& robot);

private:
    std::unordered_map<int, ros::Publisher> m_pubs;
    //ros::Publisher m_activeRobots_pub;
    std::vector<comm_msg::active_robot> m_activeRobots;
    
};

UdpToTopicEntry<Beacon,nav_msgs::Odometry>::UdpToTopicEntry(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name) : EntryPoint(udpPeer, name)
{
    comm_msg::active_robot robot;
    robot.nb_robot = 0;
    robot.ip = "";
    robot.last_time_active.sec = 0;
    robot.last_time_active.nsec = 0;

    m_activeRobots.resize(4, robot);
}

void UdpToTopicEntry<Beacon,nav_msgs::Odometry>::execute(google::protobuf::Message &msg)
{
    Beacon &m = dynamic_cast<Beacon&>(msg);
    std::shared_ptr<nav_msgs::Odometry> rosMsgOdom;

    ProtobufToRos(m, rosMsgOdom);
    std::unordered_map<int, ros::Publisher>::const_iterator it = m_pubs.find(m.nb_robot());
    if(it != m_pubs.end())
    {
        it->second.publish(*rosMsgOdom);
    }
    else
    {
        std::string name = "/test/";
        name.append("odom");
        ros::Publisher pub = m_nh.advertise<nav_msgs::Odometry>(name, 1000);
        pub.publish(*rosMsgOdom);
        m_pubs[m.nb_robot()] = pub;
    }

    m_activeRobots[m.nb_robot()].nb_robot = m.nb_robot();
    Time* time = m.mutable_timestamp();
    m_activeRobots[m.nb_robot()].last_time_active.sec = time->sec();
    m_activeRobots[m.nb_robot()].last_time_active.nsec = time->nsec();
    
    /*std::shared_ptr<comm_msg::active_robots> rosMsgActiveRobots; 
    rosMsgActiveRobots[m.nb_robot()] = rosMsgActiveRobot;*/

    /*m_activeRobots_pub = m_nh.advertise<comm_msg::active_robots>("/activeRobots", 1000);
    m_activeRobots.publish(*rosMsgActiveRobots);*/
}

void UdpToTopicEntry<Beacon,nav_msgs::Odometry>::getActiveRobots(std::vector<comm_msg::active_robot>& robot)
{
    robot = m_activeRobots;
}

#endif
