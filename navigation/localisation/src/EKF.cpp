#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"

#include <cmath>

geometry_msgs::Pose2D odomRobot;
geometry_msgs::Pose2D initRobot;

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.clear();

  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_odom  = n.subscribe("/new_odom", 1000, odomCallback);

  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //choisir une position initiale pour initialiser l'odométrie et la position initRobot

  ros::Rate r(30);

  while (ros::ok())
  {

    ros::spinOnce();

    r.sleep();
  }
}