#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Alarm.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

#include "EKF_class.h"

using namespace Eigen;

bool stand_up;

bool ReqToBool(deplacement_msg::Alarm::Request  req){
  if (req.wake_up == deplacement_msg::AlarmRequest::WAKE_UP) return true;
  else                                                       return false;
}

bool wakeUp(deplacement_msg::Alarm::Request  &req,
            deplacement_msg::Alarm::Response &res)
{
  stand_up = ReqToBool(req);
  std::cout << "stand up a la valeur " << stand_up << std::endl;
  res.stand_up = req.wake_up;
  return true;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  stand_up = false;

  ros::NodeHandle n;
  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //on choisit une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("teamColor", s, "cyan");    //à droite
  int num;
  n.param<int>("robotNumber", num, 1);             //robot 1 par défaut
 
  std::cout << s << "\n" << num << std::endl;

  EKF ekf = EKF();

  ros::Subscriber sub_odom      = n.subscribe("/new_odom", 1000, &EKF::odomCallback, &ekf);
  ros::Subscriber sub_landmarks = n.subscribe("/landmarks", 1000, &EKF::machinesCallback, &ekf);
  ros::Subscriber sub_machines  = n.subscribe("/machines", 1000, &EKF::machinesVuesCallback, &ekf);
  ros::Subscriber sub_laser     = n.subscribe("/laser", 1000, &EKF::laserCallback, &ekf);

  ros::Publisher pub_robot    = n.advertise<geometry_msgs::Point>("/robot", 1000);
  //ros::Publisher pub_machines = n.advertise<deplacement_msg::Landmarks>("/landmarks", 1000);
  //ros::Publisher pub_laser    = n.advertise<deplacement_msg::Landmarks>("/scan_global", 1000);

  ros::ServiceServer wake_up  = n.advertiseService("wake_up", wakeUp);

  ros::Rate loop_rate(10);

  //int cpt = 0;

  while (n.ok())
  {
    if(stand_up){
      ekf.set();
      ekf.fillMachines();
      ekf.prediction();
      int pos = 0, area = 0;
  
      //si on observe une machine
      if (ekf.getTabMachines().size() > 0){
        //pour toutes les machines observées
        for (auto &it : ekf.getTabMachines()){
          //on transpose la machine dans le repère global
          geometry_msgs::Pose2D m = ekf.RobotToGlobal(ekf.LaserToRobot(it));
  
          int pos = ekf.checkStateVector(m);
          if (pos != 0){
            ekf.correction(m,pos);
          }
          //else {
          //  ekf.addMachine(m,area);
          //  cpt++;
          //}
        }      
      }
  
      //std::cout << "machine(s) ajoutée(s) = " << cpt << "\n" << std::endl;
  
      VectorXd xMean = ekf.getXmean();
      //std::cout << "xMean : \n" << xMean << std::endl;
  
      ekf.printZones();
  
      geometry_msgs::Point robot;
      robot.x = xMean(0);
      robot.y = xMean(1);
  
      deplacement_msg::Landmarks m;
      for (int i = 3; i < xMean.rows(); i = i + 3){
        geometry_msgs::Pose2D md;
        md.x     = xMean(i);
        md.y     = xMean(i+1);
        md.theta = xMean(i+2);
        m.landmarks.push_back(md);
      }
  
      deplacement_msg::Landmarks l;
      for (auto &it : ekf.getScan()){
        l.landmarks.push_back(it);
      }
  
      pub_robot.publish(robot);
      //pub_machines.publish(m);
      //pub_laser.publish(l);
  
      //m.landmarks.clear();
      //l.landmarks.clear();
    }
    else {
      while(n.ok() && !stand_up){
        sleep(1);
        ros::spinOnce();
      }
    }
    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}