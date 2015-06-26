#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Alarm.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"
#include "cartographie_utils.h"

#include "EKF_class.h"

using namespace Eigen;

deplacement_msg::Landmarks tabMachines;
deplacement_msg::Landmarks scan;
geometry_msgs::Pose2D      odomRobot;
std::vector<Machine>       mps(24);
int                        count;
tf::TransformListener *g_tf_listener;

void odomCallback(const nav_msgs::Odometry& odom){
    odomRobot.x = odom.pose.pose.position.x;
    odomRobot.y = odom.pose.pose.position.y;
    odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
    tf::StampedTransform transform;
    try
    {
        g_tf_listener->lookupTransform("/map", "/laser_link",  
            machines->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    tabMachines.landmarks.clear();
    tabMachines.header.frame_id="map";
    tabMachines.header.stamp = machines->header.stamp;
    // -- Nouvelle version
    for (auto &it : machines->landmarks){
        // Changement de repère
        geometry_msgs::Pose2D p;

        double yaw = tf::getYaw(transform.getRotation());
        p.x = it.x*cos(yaw) - it.y*sin(yaw) + transform.getOrigin().x();
        p.y = it.x*sin(yaw) + it.y*cos(yaw) + transform.getOrigin().y();

        p.theta = it.theta + yaw;

        // Vérification de la zone
        int zone = machineToArea(p);
        if(zone==0)
        {
            continue;           
        }

        // Moyennage
        mps[zone-1].addX(p.x);
        mps[zone-1].addY(p.y);
        mps[zone-1].addTheta(p.theta);
        mps[zone-1].incNbActu();

        mps[zone-1].maj();

        tabMachines.landmarks.push_back(p);
    } 


    // -- Version moyennee
    // for (auto &it : machines->landmarks){
    //   geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it), odomRobot);
    //   tabMachines.landmarks.push_back(p);

    //   int zone = machineToArea(p);
    //   if(zone==0)
    //     continue;

    //   mps[zone-1].addX(p.x);
    //   mps[zone-1].addY(p.y);
    //   mps[zone-1].addTheta(p.theta);
    //   mps[zone-1].incNbActu();

    //   mps[zone-1].maj();
    // } 
}

void laserCallback(const deplacement_msg::LandmarksConstPtr& laser){
    scan.landmarks.clear();
    for (auto &it : laser->landmarks){
        geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it), odomRobot);
        scan.landmarks.push_back(p);
    }
}    

int main( int argc, char** argv )
{
    ros::init(argc, argv, "EKF_node");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);
    ros::Subscriber sub_machines = n.subscribe("/machines", 1000, machinesCallback);
    ros::Subscriber sub_laser    = n.subscribe("/laser", 1000, laserCallback);

    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/landmarks", 1000);
    ros::Publisher pub_laser    = n.advertise< deplacement_msg::Landmarks >("/scan_global", 1000);

    ros::ServiceClient client = n.serviceClient<deplacement_msg::Alarm>("wake_up");
    deplacement_msg::Alarm srv;

    count = 0;

    ros::Rate loop_rate(25);
    while (n.ok())
    { 
        deplacement_msg::Landmarks tabMPS = convert(mps);

        // count = tabMPS.landmarks.size();

        // if (count == 6/*12*/){
        //   srv.request.wake_up = 2;
        //   client.call(srv);
        // }
/*
        for (int i = 0; i < tabMPS.landmarks.size(); i++){
            std::cout << "machine (" << tabMPS.landmarks[i].x << "," << tabMPS.landmarks[i].y << ")" << std::endl;
        }
*/
        pub_machines.publish(tabMPS);
        //pub_machines.publish(tabMachines);
        // pub_laser.publish(scan);

        //tabMachines.landmarks.clear();
        // if (count >= 6){
        //   for (auto &it : mps){
        //     if (it.getCentre().x != 0 && it.getCentre().y != 0){
        //       std::cout << "machine (" << it.getCentre().x << "," << it.getCentre().y << "," << it.getCentre().theta << ")" << std::endl;
        //     }
        //   }
        // }

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}