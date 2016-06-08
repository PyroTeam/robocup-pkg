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


using namespace Eigen;

deplacement_msg::Landmarks g_walls;
deplacement_msg::Landmarks g_machines;

std::vector<Machine>       g_mps(24);
std::list<Segment>         g_sgtArray;

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
        geometry_msgs::Pose2D center;

        double yaw = tf::getYaw(transform.getRotation());
        center.x     = it.x*cos(yaw) - it.y*sin(yaw) + transform.getOrigin().x();
        center.y     = it.x*sin(yaw) + it.y*cos(yaw) + transform.getOrigin().y();
        center.theta = it.theta + yaw;

        // Vérification de la zone
        int zone = machineToArea(center);
        if(zone==0)
        {
            continue;
        }

        // Moyennage si pas de résultat aberrant ou si 1ère fois
        if (g_mps[zone-1].canBeUpdated(center))
        {
            g_mps[zone-1].update(center);
        }
    }

    g_machines = convert(g_mps);
}

void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments)
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
    g_tf_listener->lookupTransform("map", tf_prefix+"laser_link", segments->header.stamp, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  g_walls.landmarks.clear();
  g_walls.header.frame_id=tf_prefix+"laser_link";
  g_walls.header.stamp = segments->header.stamp;


  for (int i = 0; i < segments->landmarks.size(); i = i+2)
  {
    // Changement de repère
    geometry_msgs::Pose2D p, q;

    double yaw = tf::getYaw(transform.getRotation());

    p.x     = segments->landmarks[i].x*cos(yaw) - segments->landmarks[i].y*sin(yaw) + transform.getOrigin().x();
    p.y     = segments->landmarks[i].x*sin(yaw) + segments->landmarks[i].y*cos(yaw) + transform.getOrigin().y();
    p.theta = segments->landmarks[i].theta + yaw;

    q.x     = segments->landmarks[i+1].x*cos(yaw) - segments->landmarks[i+1].y*sin(yaw) + transform.getOrigin().x();
    q.y     = segments->landmarks[i+1].x*sin(yaw) + segments->landmarks[i+1].y*cos(yaw) + transform.getOrigin().y();
    q.theta = segments->landmarks[i+1].theta + yaw;

    Segment seg;
    seg.setPoints(pose2DToPoint(p), pose2DToPoint(q));

    // si le segment est un mur
    if (seg.isAWall())
    {
        g_walls.landmarks.push_back(p);
        g_walls.landmarks.push_back(q);
    }
  }

  std::list<Segment> tmp = landmarksToSegments(g_walls);
  adjust(g_sgtArray,tmp);
  gather(g_sgtArray);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    ros::Subscriber sub_segments = n.subscribe("objectDetection/segments", 100, segmentsCallback);
    ros::Subscriber sub_machines = n.subscribe("objectDetection/machines", 100, machinesCallback);

    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("objectDetection/landmarks", 100);
    ros::Publisher pub_segments_global = n.advertise< deplacement_msg::Landmarks >("objectDetection/segments_global", 100);

    ros::Rate loop_rate (10);
    while(n.ok())
    {
      pub_machines.publish(convert(g_mps));
      pub_segments_global.publish(backToLandmarks(g_sgtArray));

      // Spin
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
