#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "cartographie_utils.h"

#include "EKF_class.h"

using namespace Eigen;

deplacement_msg::Landmarks g_tabMachines;
deplacement_msg::Landmarks g_tabSegments;
geometry_msgs::Pose2D      g_odomRobot;
std::vector<Machine>       g_mps(24);
std::vector<Segment>       g_sgtArray;
tf::TransformListener     *g_tf_listener;
double                     g_angular_speed;
double                     g_linear_speed;

void odomCallback(const nav_msgs::Odometry& odom)
{
    g_odomRobot.x = odom.pose.pose.position.x;
    g_odomRobot.y = odom.pose.pose.position.y;
    g_odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);

    g_angular_speed = odom.twist.twist.angular.z;
    g_linear_speed  = odom.twist.twist.linear.x;
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{

    tf::StampedTransform transform;
    try
    {
        g_tf_listener->lookupTransform("/map", "/laser_link", machines->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    g_tabMachines.landmarks.clear();
    g_tabMachines.header.frame_id="/laser_link";
    g_tabMachines.header.stamp = machines->header.stamp;

    for (auto &it : machines->landmarks)
    {
        // Changement de repère
        geometry_msgs::Pose2D p;
        //geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it), g_odomRobot);

        double yaw = tf::getYaw(transform.getRotation());
        p.x     = it.x*cos(yaw) - it.y*sin(yaw) + transform.getOrigin().x();
        p.y     = it.x*sin(yaw) + it.y*cos(yaw) + transform.getOrigin().y();
        p.theta = it.theta + yaw;

        // Vérification de la zone
        int zone = machineToArea(p);
        if(zone==0)
        {
            continue;           
        }

        // Moyennage si pas de résultat aberrant ou si 1ère fois
        if (g_mps[zone-1].getNbActu() == 0 ||
           (std::abs(g_mps[zone-1].getCentre().x - p.x) <= 0.2 &&
            std::abs(g_mps[zone-1].getCentre().y - p.y) <= 0.2 &&
            std::abs(g_mps[zone-1].getCentre().theta - p.theta) <= 0.34))
        {
            g_mps[zone-1].addX(p.x);
            g_mps[zone-1].addY(p.y);
            g_mps[zone-1].addTheta(p.theta);
            g_mps[zone-1].incNbActu();

            g_mps[zone-1].maj();
        }

        g_tabMachines.landmarks.push_back(p);
    } 
}

void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments)
{
    g_tabSegments.landmarks.clear();
    g_tabSegments.header.frame_id="/laser_link";
    g_tabSegments.header.stamp = segments->header.stamp;

    for (int i = 0; i < segments->landmarks.size(); i = i+2)
    {
        // Changement de repère
        geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(segments->landmarks[i]), g_odomRobot);
        geometry_msgs::Pose2D q = RobotToGlobal(LaserToRobot(segments->landmarks[i+1]), g_odomRobot);

        if (std::abs(p.x) <= 6.5 && std::abs(p.y - 2.0) <= 4.0 &&
            std::abs(q.x) <= 6.5 && std::abs(q.y - 2.0) <= 4.0)
        {
            g_tabSegments.landmarks.push_back(p);
            g_tabSegments.landmarks.push_back(q);
        }
    }
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);
    ros::Subscriber sub_machines = n.subscribe("/machines", 1000, machinesCallback);
    ros::Subscriber sub_segments = n.subscribe("/segments", 1000, segmentsCallback);

    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/landmarks", 1000);
    ros::Publisher pub_segments_global = n.advertise< deplacement_msg::Landmarks >("/segments_global", 1000);

    ros::Rate loop_rate(10);
    while (n.ok())
    { 
        pub_machines.publish(convert(g_mps));

        std::vector<Segment> tmp = landmarksToSegments(g_tabSegments);
        maj(g_sgtArray,tmp);
        pub_segments_global.publish(backToLandmarks(g_sgtArray));
        //tmp.clear();
        g_sgtArray.clear();

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}