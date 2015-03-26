#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#include <cmath>

geometry_msgs::Pose2D odomRobot;
geometry_msgs::Pose2D initRobot;
geometry_msgs::Pose2D cmdVel;

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.clear();

  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  cmdVel.clear();

  cmdVel.x     = cmd_vel.linear.x;
  cmdVel.y     = cmd_vel.linear.y;
  cmdVel.theta = cmd_vel.angular.z;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_cmd_vel  = n.subscribe("/cmd_vel",  1000, cmdVelCallback);
  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);

  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //choisir une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("Team_Color", s, "cyan");   //à droite
  int n;
  n.param<int>("Robot_Number", n, "1");    //robot 1 par défaut

  initPosRobot(s,n);

  ros::Rate r(30);

  while (ros::ok())
  {

    ros::spinOnce();

    r.sleep();
  }
}