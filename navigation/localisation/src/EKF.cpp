#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <cmath>

geometry_msgs::Pose2D odomRobot;
geometry_msgs::Pose2D initRobot;
geometry_msgs::Pose2D cmdVel;

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  cmdVel.x     = cmd_vel.linear.x;
  cmdVel.y     = cmd_vel.linear.y;
  cmdVel.theta = cmd_vel.angular.z;
}

void initPosRobot(std::string s,int n){
  int color = 0;
  if (s == "cyan"){
    color = 1;
  }
  else {
    color = -1;
  }

  initRobot.theta =  0.0;
  initRobot.y   = -0.5;

  switch(n){
    case 1 :
      initRobot.x = color*3.5;
    break;
    case 2 :
      initRobot.x = color*4.5;
    break;
    case 3 :
      initRobot.x = color*5.5;
    break;
    default :
    break;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_cmd_vel  = n.subscribe("/cmd_vel",  1000, cmdVelCallback);
  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);

  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //on choisit une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("Team_Color", s, "cyan");    //à droite
  int num;
  n.param<int>("Robot_Number", num, 1);             //robot 1 par défaut

  initPosRobot(s,num);

  std::cout << "Machine ("<< initRobot.x << ", " << initRobot.y << ")" << std::endl;
  std::cout << "\n" << std::endl;

  ros::Rate loop_rate (5);

  while (ros::ok())
  {

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}