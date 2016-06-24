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
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ExplorationSignal.h"

deplacement_msg::Machines  g_machines;
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
        g_tf_listener->waitForTransform("map",tf_prefix+"laser_link",machines->header.stamp /*+ ros::Duration(0.1)*/,ros::Duration(1.0));
        g_tf_listener->lookupTransform("map", tf_prefix+"laser_link", machines->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    g_machines.landmarks.clear();
    g_machines.header.frame_id="map";
    g_machines.header.stamp = machines->header.stamp;

    for (auto &it : machines->landmarks)
    {
        // Changement de repère
        geometry_msgs::Pose2D center = geometry_utils::changeFrame(it, transform);

        // Vérification de la zone
        int zone = common_utils::getArea(center);

        // Si la machine est bien dans une zone
        if (zone != 0)
        {
            g_mps[zone-1].update(center);
            g_mps[zone-1].zone(zone);

            // Ajout reverse
            geometry_msgs::Pose2D reverse = g_mps[zone-1].reversePose();
            zone = common_utils::getArea(reverse);
            if (zone != 0 && g_mps[zone-1].neverSeen())
            {
                g_mps[zone-1].update(reverse);
                g_mps[zone-1].zone(zone);
            }
        }
    }
}

void artagCallback(const ar_track_alvar_msgs::AlvarMarkers& artags)
{
  std::vector<ar_track_alvar_msgs::AlvarMarker> tmp;
  tmp = artags.markers;

  static ros::NodeHandle nh;
  std::string tf_prefix;
  nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
  if (tf_prefix.size() != 0)
  {
      tf_prefix += "/";
  }

  for (int i = 0; i < tmp.size(); i++)
  {
    tmp[i].pose.header.frame_id = tmp[i].header.frame_id;

    geometry_msgs::PoseStamped pose_map;
    try
    {
      g_tf_listener->waitForTransform("map",tf_prefix+"tower_camera_link",artags.header.stamp,ros::Duration(1.0));
      g_tf_listener->transformPose("map",tmp[i].pose,pose_map);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transform exception : %s",ex.what());
    }

    for (auto &it2 : g_mps)
    {
      // si la machine a déjà été vue au moins une fois et que
      // l'ar tag est assez proche pour considérer qu'il est bien celui de la machine et que
      // l'angle n'est pas bon
      // Oui magic number mais ya un moment faut arrêter quoi
      if (!it2.neverSeen() &&
          geometry_utils::distance(pose_map.pose.position, it2.getCentre()) <= 0.5 &&
          ((tmp[i].id%2 == 1 && it2.getCentre().theta < 0) || (tmp[i].id%2 == 0 && it2.getCentre().theta > 0)))
      {
          ROS_INFO("Switch sides of Machine in zone %d", it2.zone());
          it2.switchSides();
      }
    }
  }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_machines = n.subscribe("objectDetection/machines", 1, machinesCallback);
    ros::Subscriber sub_artag    = n.subscribe("computerVision/ar_pose_marker", 1, artagCallback);

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
