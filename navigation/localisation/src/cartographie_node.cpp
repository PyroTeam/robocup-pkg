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
#include "comm_msg/ExplorationZone.h"

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
        g_tf_listener->waitForTransform("map",tf_prefix+"laser_link",machines->header.stamp + ros::Duration(0.1),ros::Duration(1.0));
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

void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& artags)
{
  // transfo tf de tower_camera_link vers map à faire !
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
      g_tf_listener->waitForTransform("map",tf_prefix+"tower_camera_link",artags->header.stamp + ros::Duration(0.1),ros::Duration(1.0));
      g_tf_listener->lookupTransform("map", tf_prefix+"tower_camera_link", artags->header.stamp, transform);
  }
  catch (tf::TransformException ex)
  {
      ROS_WARN("%s",ex.what());
      return;
  }

  for (auto &it : artags->markers)
  {
  /*
    geometry_msgs::PoseStamped pose_map;
    try
    {
      g_tf_listener->transformPose("map",it.pose,pose_map);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transform exception : %s",ex.what());
    }
*/
    tf::Quaternion q(it.pose.pose.orientation.x, it.pose.pose.orientation.y, it.pose.pose.orientation.z, it.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

		ROS_INFO("I see: [%d] at (%f,%f) with RPY (%f, %f, %f)", it.id, it.pose.pose.position.x, it.pose.pose.position.y, roll, pitch, yaw);
    for (auto &it2 : g_mps)
    {
      // si l'ar tag est assez proche pour considérer qu'il est celui sur la machine
      // Oui magic number mais ya un moment faut arrêter quoi
      if (geometry_utils::distance(it.pose.pose.position, it2.getCentre()) <= 0.5)
      {
        ROS_INFO("I detect an AR Tag corresponding to a known machine");
        // ar tag impair = INPUT
        if (it.id%2 == 1)
        {
          ROS_INFO("I see an input");
        }
        else
        {
          ROS_INFO("I see an output");
        }
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
    ros::Subscriber sub_zones    = n.subscribe("refBoxComm/ExplorationInfo", 1, zonesCallback);
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
