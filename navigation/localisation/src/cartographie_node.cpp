#include <ros/ros.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include "deplacement_msg/Landmarks.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "cartographie_utils.h"
#include "math_functions.h"

using namespace Eigen;
using namespace message_filters;

deplacement_msg::Landmarks g_walls;
deplacement_msg::Landmarks g_machines;
deplacement_msg::Landmarks g_tabMachines;

geometry_msgs::Pose2D      g_odomRobot;
std::vector<Machine>       g_mps(24);
std::list<Segment>         g_sgtArray;

tf::TransformListener     *g_tf_listener;
/*
void odomCallback(const nav_msgs::Odometry& odom)
{
    static ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    tf::StampedTransform transform;
    try
    {
        g_tf_listener->lookupTransform("/map", tf_prefix+"odom", odom.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    g_odomRobot.x = odom.pose.pose.position.x;
    g_odomRobot.y = odom.pose.pose.position.y;
    g_odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{
    static ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    tf::StampedTransform transform;
    try
    {
        g_tf_listener->lookupTransform("/map", tf_prefix+"laser_link", machines->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    g_tabMachines.landmarks.clear();
    g_tabMachines.header.frame_id=tf_prefix+"laser_link";
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
*/
void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments)
{
  static ros::NodeHandle nh;
  std::string tf_prefix;
  nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
  if (tf_prefix.size() != 0)
  {
    tf_prefix += "/";
  }

  tf::StampedTransform transform;
  try
  {/*
    if (g_tf_listener->waitForTransform(tf_prefix+"odom", tf_prefix+"laser_link", segments->header.stamp, ros::Duration(3.0)))
    {*/
        g_tf_listener->lookupTransform(tf_prefix+"odom", tf_prefix+"laser_link", segments->header.stamp, transform);
    /*}
    else
    {
      ROS_WARN("Unable to get TF transform");
      return;
    }*/
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

    g_walls.landmarks.push_back(p);
    g_walls.landmarks.push_back(q);
  /*
    // si le segment est un mur

    if ((((std::abs(p.x + 6.0) <= 0.3 && std::abs(q.x + 6.0) <= 0.3) ||
          (std::abs(p.x - 6.0) <= 0.3 && std::abs(q.x - 6.0) <= 0.3)) &&
           std::abs(p.y - 3.0) <= 3.3 && std::abs(q.y - 3.0) <= 3.3) ||
        (((std::abs(p.y - 6.0) <= 0.3 && std::abs(q.y - 6.0) <= 0.3) ||
          (std::abs(p.y) <= 0.3 && std::abs(q.y) <= 0.3)) &&
           std::abs(p.x) <= 6.3 && std::abs(q.x) <= 6.3))
    {
        g_walls.landmarks.push_back(p);
        g_walls.landmarks.push_back(q);
    }*/
  }

  std::list<Segment> tmp = landmarksToSegments(g_walls);
  adjust(g_sgtArray,tmp);
  gather(g_sgtArray);

  ros::Publisher pub_segments_global = nh.advertise< deplacement_msg::Landmarks >("objectDetection/segments_global", 1000);
  pub_segments_global.publish(backToLandmarks(g_sgtArray));
}
#if 0
void callback(const nav_msgs::OdometryConstPtr& odom, const deplacement_msg::LandmarksConstPtr& segments)
{
  // Solve all of perception here...

  static ros::NodeHandle nh;
  std::string tf_prefix;
  nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
  if (tf_prefix.size() != 0)
  {
    tf_prefix += "/";
  }

  tf::StampedTransform transform;
  try
  {
    g_tf_listener->lookupTransform(tf_prefix+"odom", tf_prefix+"laser_link", segments->header.stamp, transform);
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

    g_walls.landmarks.push_back(p);
    g_walls.landmarks.push_back(q);

    // si le segment est un mur

    if ((((std::abs(p.x + 6.0) <= 0.3 && std::abs(q.x + 6.0) <= 0.3) ||
          (std::abs(p.x - 6.0) <= 0.3 && std::abs(q.x - 6.0) <= 0.3)) &&
           std::abs(p.y - 3.0) <= 3.3 && std::abs(q.y - 3.0) <= 3.3) ||
        (((std::abs(p.y - 6.0) <= 0.3 && std::abs(q.y - 6.0) <= 0.3) ||
          (std::abs(p.y) <= 0.3 && std::abs(q.y) <= 0.3)) &&
           std::abs(p.x) <= 6.3 && std::abs(q.x) <= 6.3))
    {
        g_walls.landmarks.push_back(p);
        g_walls.landmarks.push_back(q);
    }
  }

  std::list<Segment> tmp = landmarksToSegments(g_walls);
  adjust(g_sgtArray,tmp);
  gather(g_sgtArray);

  ros::Publisher pub_segments_global = nh.advertise< deplacement_msg::Landmarks >("objectDetection/segments_global", 1000);
  pub_segments_global.publish(backToLandmarks(g_sgtArray));
}
#endif

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Cartographie");
    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;

    ros::NodeHandle n;

    //ros::Subscriber sub_odom     = n.subscribe("objectDetection/new_odom", 1000, odomCallback);
    ros::Subscriber sub_segments = n.subscribe("objectDetection/segments", 1000, segmentsCallback);
    // TODO: Ensure that the good topic is subscribed below
    //ros::Subscriber sub_machines = n.subscribe("objectDetection/segments", 1000, machinesCallback);

    //ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("objectDetection/landmarks", 1000);
    //ros::Publisher pub_segments_global = n.advertise< deplacement_msg::Landmarks >("objectDetection/segments_global", 1000);
/*
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "hardware/odom", 100);
    message_filters::Subscriber<deplacement_msg::Landmarks> sub_segments(n, "objectDetection/segments", 100);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, deplacement_msg::Landmarks> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_odom, sub_segments);
    sync.registerCallback(boost::bind(&callback, _1, _2));
*/
    ///////////////////////////////////////// TO DO ////////////////////////////////
    // Trouve les machines
    //std::vector<Machine> listOfMachines = recognizeMachinesFrom(listOfSegments);

    //pub_machines.publish(convert(g_mps));
/*
    std::list<Segment> tmp = landmarksToSegments(g_walls);
    adjust(g_sgtArray,tmp);
    gather(g_sgtArray);
    std::cout << g_sgtArray.size() << std::endl;
    pub_segments_global.publish(backToLandmarks(g_sgtArray));*/
    //tmp.clear();
    //g_sgtArray.clear();

    // Spin
    ros::spin();

    return 0;
}
