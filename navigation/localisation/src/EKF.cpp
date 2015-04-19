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

VectorXd xMean;
MatrixXd P;

Vector3d xPredicted;
Vector3d cmdVel;

std::vector<geometry_msgs::Pose2D> tabMachines;
ros::Time temps;

void init(std::string s, int num){
  /*int color = 0;
  if (s == "cyan"){
    color = 1;
  }
  else {
    color = -1;
  }

  m_initRobot.theta =  0.0;
  m_initRobot.y   = -0.5;

  switch(n){
    case 1 :
      m_initRobot.x = color*3.5;
    break;
    case 2 :
      m_initRobot.x = color*4.5;
    break;
    case 3 :
      m_initRobot.x = color*5.5;
    break;
    default :
    break;
  }*/

  initRobot.x     = 0.0;
  initRobot.y     = 0.0;
  initRobot.theta = 0.0;

  xMean.conservativeResize(3);
  xMean.setZero();
  P.conservativeResize(3,3);
  P.setZero();

  temps = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);

  cmdVel(0) = odom.twist.twist.linear.x;
  cmdVel(1) = odom.twist.twist.linear.y;
  cmdVel(2) = odom.twist.twist.angular.z;

  std::cout << "cmdVel = \n" << cmdVel << std::endl;
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

    geometry_msgs::Pose2D pDansRepereGlobal = RobotToGlobal(p,initRobot,odomRobot);
    if (pDansRepereGlobal.x > -6.0 &&
        pDansRepereGlobal.x <  6.0 &&
        pDansRepereGlobal.y >  0.0 &&
        pDansRepereGlobal.y <  6.0)
    {
      tabMachines.push_back(pDansRepereGlobal);
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, &odomCallback);
  ros::Subscriber sub_machines = n.subscribe("/machines", 1000, &machinesCallback);

  ros::Publisher pub_robot = n.advertise<geometry_msgs::Point>("/robot", 1000);
  ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/m", 1000);

  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //on choisit une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("Team_Color", s, "cyan");    //à droite
  int num;
  n.param<int>("Robot_Number", num, 1);             //robot 1 par défaut

  init(s,num);

  ros::Rate loop_rate(5);

  int cpt = 0;

  while (ros::ok())
  {
    xPredicted = prediction(xMean, P, cmdVel, temps);
    int pos = 0;
    //std::cout << "odométrie du robot : \n" << odomRobot << "\n" << std::endl;

    //si on observe une machine
    if (tabMachines.size() > 0/* && cpt < 12*/){
      for (auto &it : tabMachines){
        //si le vecteur d'état contient déjà des machines
        if (xMean.rows() > 3){
          //on cherche une correspondance
          pos = checkStateVector(xMean, it);
          //s'il n'y en a pas
          if (pos == 0){
            //std::cout << "la machine n'existe pas \n" << std::endl;
            addMachine(it, xMean, P);
            cpt++;
            //std::cout << "ajout machine\n" << std::endl;
          }
          else {
            //std::cout << "la machine existe en position " << pos/3 << "\n" << std::endl;
            correction(xMean, P, xPredicted, pos);
            //std::cout << "xMean : \n" << xMean << "\n" << std::endl;
          }
        }
        else{
          //std::cout << "il n'y a jamais eu de machine \n" << std::endl;
          addMachine(it, xMean, P);
          cpt++;
          //std::cout << "ajout machine\n" << std::endl;
        }
      }      
    }

    std::cout << "vecteur d'état : \n" << xMean << "\n" << std::endl;
    geometry_msgs::Point robot;
    robot.x = xMean(0);
    robot.y = xMean(1);

    deplacement_msg::Landmarks m;
    for (int i = 3; i < xMean.rows(); i = i + 3){
      geometry_msgs::Pose2D md;
      md.x = xMean(i);
      md.y = xMean(i+1);
      m.landmarks.push_back(md);
    }

    pub_robot.publish(robot);
    pub_machines.publish(m);

    m.landmarks.clear();

    std::cout << "machine(s) ajoutée(s) = " << cpt << "\n" << std::endl;

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}