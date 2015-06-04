#include <functional>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "udpPeer.h"
#include "encryptUtils.h"
#include "topicToUdpEntry.h"
#include "udpToTopicEntry.h"
#include "udpToTopicBeacon.h"
#include "messageCatalog.h"

#include "comm_msg/activity.h"

#include <initializer_list>
#include <vector>

// Variables globales
geometry_msgs::Pose2D g_pose;
ros::Time g_time;

void PoseCallback(const nav_msgs::Odometry &odom);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "InterRobotComm");
    ros::NodeHandle nh;

    //chargement de la configuration
    int robotNumber;
    nh.param<int>("robotNumber", robotNumber, 0);
    int portIn = 5001;
    nh.param<int>("portIn", portIn, 5001);
    int portOut = 5001;
    nh.param<int>("portOut", portOut, 5001);
    std::string adresseIP;
    nh.param<std::string>("adresseIP", adresseIP, "127.0.0.255");
    // Fin du chargement de la configuration

    boost::asio::io_service io_service;
    std::shared_ptr<UdpPeer> udpPeer(new UdpPeer(io_service, portOut, portIn, adresseIP));

    std::shared_ptr<MessageCatalog> msgCatalog(new(MessageCatalog));
    msgCatalog->add<Activity>();
    msgCatalog->add<Beacon>();
    udpPeer->setCatalog(msgCatalog);

    UdpToTopicEntry<Activity, comm_msg::activity> udpToTopicActivity(udpPeer, "activity");
    TopicToUdpEntry<comm_msg::activity> topicToUdpActivity(udpPeer, "/activity");

    std::shared_ptr<MessageDispatcher> msgDispatcher(new(MessageDispatcher));
    msgDispatcher->Add<Activity>(std::function<void(google::protobuf::Message&)>(
        boost::bind(&UdpToTopicEntry<Activity, comm_msg::activity>::execute, &udpToTopicActivity, _1)));

    /* Traitement message beacon */
    ros::Subscriber pose_sub = nh.subscribe("/odom", 1000, &PoseCallback);
    ros::Rate loop_rate(1);

    g_pose.x = 0;
    g_pose.y = 0;
    g_pose.theta = 0;
    g_time = ros::Time::now();

    std::shared_ptr<Beacon> beacon(new Beacon);
    std::shared_ptr<google::protobuf::Message> proto_msg;
    proto_msg = beacon;

    UdpToTopicEntry<Beacon, nav_msgs::Odometry> udpToTopicBeacon(udpPeer, "beacon");
    msgDispatcher->Add<Beacon>(std::function<void(google::protobuf::Message&)>(
        boost::bind(&UdpToTopicEntry<Beacon, nav_msgs::Odometry>::execute, &udpToTopicBeacon, _1)));
    udpPeer->setDispatcher(msgDispatcher);

    comm_msg::active_robots rosMsgActiveRobots;
    ros::Publisher activeRobots_pub = nh.advertise<comm_msg::active_robots>("/activeRobots", 1000);
    /* Fin du traitement du message beacon */

    while(ros::ok())
    {
        Time* time = beacon->mutable_timestamp();
        time->set_sec(g_time.sec);
        time->set_nsec(g_time.nsec);
        beacon->set_nb_robot(robotNumber);
        Pose2D* pose = beacon->mutable_pose();
        pose->set_x(g_pose.x);
        pose->set_y(g_pose.y);
        pose->set_theta(g_pose.theta);

        udpPeer->send(proto_msg);

        udpToTopicBeacon.getActiveRobots(rosMsgActiveRobots.robot);
        activeRobots_pub.publish(rosMsgActiveRobots);

		io_service.poll();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void PoseCallback(const nav_msgs::Odometry &odom)
{
    g_pose.x = odom.pose.pose.position.x;
    g_pose.y = odom.pose.pose.position.y;
    g_pose.theta = tf::getYaw(odom.pose.pose.orientation);

    g_time = ros::Time::now();
}
