#include <ros/ros.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include "deplacement_msg/Landmarks.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "cartographie_utils.h"
#include "math_functions.h"

deplacement_msg::Landmarks g_machines;
std::vector<Machine>       g_mps(24);

tf::TransformListener     *g_tf_listener;

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{
    static ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    tf::StampedTransform transform;
    try
    {
        g_tf_listener->lookupTransform("map", tf_prefix+"laser_link", machines->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    g_machines.landmarks.clear();
    g_machines.header.frame_id=tf_prefix+"laser_link";
    g_machines.header.stamp = machines->header.stamp;

    for (auto &it : machines->landmarks)
    {
        // Changement de repère
        geometry_msgs::Pose2D center = changeFrame(it, transform);

        // Vérification de la zone
        int zone = getArea(center);

        // Si la machine est bien dans une zone et peut être mise à jour
        if (zone != 0 && g_mps[zone-1].canBeUpdated(center))
        {
            g_mps[zone-1].update(center);
        }
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_machines = n.subscribe("objectDetection/machines", 100, machinesCallback);

    ros::Publisher pub_machines        = n.advertise< deplacement_msg::Landmarks >("objectDetection/landmarks", 1);

    ros::Rate loop_rate (10);
    while(n.ok())
    {
      g_machines.landmarks = convert(g_mps);

      pub_machines.publish(g_machines);

      // Spin
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
