#include <ros/ros.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <common_utils/zone.h>
#include <common_utils/types.h>

#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Machines.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "geometry_utils.h"
#include "math_functions.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#define CIRCUM_MACHINE_RADIUS 0.35
#define ACCEPTANCE_THRESHOLD 0.1

deplacement_msg::Machines  g_machines;
std::vector<Machine>       g_mps(24);

using namespace common_utils;

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

  // TODO: adapter en fonction de la croyance en la position donnée par la localisation
  tf::StampedTransform transform;
  try
  {
    g_tf_listener->waitForTransform("map",tf_prefix+"laser_link",machines->header.stamp,ros::Duration(1.0));
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
    if (zone != 0 && g_mps[zone-1].isInsideZone(center, zone))
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

static bool isInput(int id)
{
  return id%2 == 1;
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

  // TODO: adapter en fonction de la croyance en la position donnée par la localisation
  for (int i = 0; i < tmp.size(); i++)
  {
    tmp[i].pose.header.frame_id = tmp[i].header.frame_id;

    geometry_msgs::PoseStamped pose_map;
    if (g_tf_listener->waitForTransform("map",tf_prefix+"tower_camera_link",artags.header.stamp,ros::Duration(1.0)))
    {
      g_tf_listener->transformPose("map",tmp[i].pose,pose_map);
    }
    else
    {
      ROS_ERROR("Transform EXCEPTION from tower_cam to map");
    }
    geometry_msgs::PoseStamped poseArTagInRobotFrame;
    if (g_tf_listener->waitForTransform(tf_prefix+"base_link",tf_prefix+"tower_camera_link",artags.header.stamp,ros::Duration(1.0)))
    {
      g_tf_listener->transformPose(tf_prefix+"base_link",tmp[i].pose,poseArTagInRobotFrame);
    }
    else
    {
      ROS_ERROR("Transform EXCEPTION from tower_cam to base");
    }

    double dist = sqrt(poseArTagInRobotFrame.pose.position.x * poseArTagInRobotFrame.pose.position.x +
      poseArTagInRobotFrame.pose.position.y * poseArTagInRobotFrame.pose.position.y);

      const double max_distance = 5.0;

      for (auto &it2 : g_mps)
      {
        // si la machine n'a pas été corrigée en angle et que
        // l'ar tag est assez proche pour considérer qu'il est bien celui de la machine
        if (exists(tmp[i].id) && dist <= max_distance && !it2.orientationOk() &&
        geometry_utils::distance(pose_map.pose.position, it2.getCentre()) <= CIRCUM_MACHINE_RADIUS)
        {
          ROS_INFO("I see ID %d corresponding to machine (%f) in zone %d having the angle %f", tmp[i].id, it2.getCentre().theta, it2.zone(), geometry_utils::normalizeAngle(tf::getYaw(pose_map.pose.orientation)+M_PI/2));

          // normalisation [0, 2 PI]
          double angleMachine = geometry_utils::normalizeAngle(it2.getCentre().theta, 0.0, 2*M_PI);
          it2.setTheta(angleMachine);
          // normalisation [0, 2 PI]
          double angleARTag = geometry_utils::normalizeAngle(tf::getYaw(pose_map.pose.orientation), 0.0, 2*M_PI);
          // ecart en angle
          double diff = angleMachine - angleARTag;
          // normalisation [0, 2 PI]
          double norm = geometry_utils::normalizeAngle(diff, 0.0, 2*M_PI);

          if (isInput(tmp[i].id))
          {
            it2.setId(tmp[i].id);
            if (norm < M_PI_2)
            {
              ROS_DEBUG("Angle is good :D");
              it2.orientation(true);
            }
            else
            {
              it2.switchSides();
              ROS_INFO("So I switch the machine angle to %f", it2.getCentre().theta);
            }
          }
          else
          {
            it2.setId(tmp[i].id-1);
            if (norm > M_PI_2)
            {
              ROS_DEBUG("Angle is good :D");
              it2.orientation(true);
            }
            else
            {
              it2.switchSides();
              ROS_INFO("So I switch the machine angle to %f", it2.getCentre().theta);
            }
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
    ros::Subscriber sub_artag    = n.subscribe("computerVision/ar_pose_marker", 1, artagCallback);

    ros::Publisher pub_machines = n.advertise< deplacement_msg::Machines >("objectDetection/landmarks", 1);

    ros::Rate loop_rate (30);
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
