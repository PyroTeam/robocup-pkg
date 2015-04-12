#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "EKF_functions.h"

using namespace Eigen;

geometry_msgs::Pose2D odomRobot;
geometry_msgs::Pose2D initRobot;
geometry_msgs::Pose2D cmdVel;
std::vector<geometry_msgs::Pose2D> tabMachines;
ros::Time temps;

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);

  cmdVel.x     = odom.twist.twist.linear.x;
  cmdVel.y     = odom.twist.twist.linear.y;
  cmdVel.theta = odom.twist.twist.angular.z;
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  tabMachines.clear();
  for (auto &it : machines->landmarks){
    geometry_msgs::Pose2D posLaser;
    posLaser.x     = it.x;
    posLaser.y     = it.y;
    posLaser.theta = it.theta;
    //changement de base vers le repère du robot
    geometry_msgs::Pose2D p = LaserToRobot(posLaser);
    //changment de base dans le repère global
    geometry_msgs::Pose2D pDansRepereGlobal = RobotToGlobal(p, initRobot, odomRobot);
    tabMachines.push_back(pDansRepereGlobal);
  }
}

void initPosRobot(std::string s,int n){
  /*int color = 0;
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
  }*/

  initRobot.x     = 0.0;
  initRobot.y     = 0.0;
  initRobot.theta = 0.0;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);
  ros::Subscriber sub_machines = n.subscribe("/machines", 1000, machinesCallback);

  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //on choisit une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("Team_Color", s, "cyan");    //à droite
  int num;
  n.param<int>("Robot_Number", num, 1);             //robot 1 par défaut

  initPosRobot(s,num);

  ros::Rate loop_rate(1);

  VectorXd xMean(3);
  xMean(0) = initRobot.x;
  xMean(1) = initRobot.y;
  xMean(2) = initRobot.theta;

  MatrixXd P(3,3);
  P.setZero();

  Vector3d xPredicted;

  while (ros::ok())
  {
    xPredicted = prediction(xMean, P, cmdVel, temps);

    //si on observe une machine
    if (tabMachines.size() > 0){
      int i = chooseMachine(tabMachines, xMean);
      correction(xMean, P, xPredicted, tabMachines[i]);
    }

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}