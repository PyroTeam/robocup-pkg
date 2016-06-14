#include <ros/ros.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <common_utils/zone.h>

#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Machines.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "geometry_utils.h"
#include "math_functions.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ExplorationSignal.h"
#include "comm_msg/ExplorationZone.h"

deplacement_msg::Machines g_machines;
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
        g_tf_listener->waitForTransform("map",tf_prefix+"laser_link",machines->header.stamp + ros::Duration(0.1),ros::Duration(1.0));
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
        geometry_msgs::Pose2D center = geometry_utils::changeFrame(it, transform);

        // Vérification de la zone
        int zone = common_utils::getArea(center);

        // Si la machine est bien dans une zone et peut être mise à jour
        if (zone != 0 && g_mps[zone-1].canBeUpdated(center))
        {
            g_mps[zone-1].update(center);
        }

        // Ajout reverse
        geometry_msgs::Pose2D reverse = g_mps[zone-1].reversePose();
        zone = common_utils::getArea(reverse);
        if (zone != 0 && g_mps[zone-1].neverSeen())
        {
            g_mps[zone-1].update(reverse);
        }
    }
}

void zonesCallback(const comm_msg::ExplorationInfo &msg)
{
  for (auto &it : msg.zones)
  {
    g_mps[it.zone].color(it.team_color);
  }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_machines = n.subscribe("objectDetection/machines", 1, machinesCallback);
    ros::Subscriber sub_zones    = n.subscribe("refBoxComm/ExplorationInfo", 1, zonesCallback);

    ros::Publisher pub_machines = n.advertise< deplacement_msg::Machines >("objectDetection/landmarks", 1);

    ros::Rate loop_rate (10);
    while(n.ok())
    {
      g_machines.landmarks = convertIntoMsg(g_mps);

      pub_machines.publish(g_machines);

      // Spin
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
