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
#include "laserScan.h"
#include "landmarks_detection_utils.h"

#include "EKF_class.h"

using namespace Eigen;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;
  //selon la couleur de l'équipe (côté du terrain) et le numéro du robot,
  //on choisit une position initiale pour initialiser l'odométrie et la position initRobot
  std::string s;
  n.param<std::string>("teamColor", s, "cyan");    //à droite
  int num;
  n.param<int>("robotNumber", num, 1);             //robot 1 par défaut
 
  std::cout << s << "\n" << num << std::endl;

  EKF ekf = EKF(s, num);

  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, &EKF::odomCallback, &ekf);
  ros::Subscriber sub_machines = n.subscribe("/machines", 1000, &EKF::machinesCallback, &ekf);
  ros::Subscriber sub_laser    = n.subscribe("/laser", 1000, &EKF::laserCallback, &ekf);

  ros::Publisher pub_robot    = n.advertise< geometry_msgs::Point >("/robot", 1000);
  ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/landmarks", 1000);
  ros::Publisher pub_laser    = n.advertise< deplacement_msg::Landmarks >("/scan_global", 1000);

  ros::Rate loop_rate(10);

  int cpt = 0;

  while (ros::ok())
  {
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
        else {
          ekf.addMachine(m,area);
          cpt++;
        }

        /*
        //on convertit la machine en zone
        int area = ekf.machineToArea(m);

        if (area != 0){
          std::cout << "machine (" << m.x << "," << m.y << ") dans zone " << area << std::endl;
        }

        //si la zone est cohérente
        if ((area != 0) && (std::abs(m.x) < 2.8) && (std::abs(m.y - 3.0) < 2.8)){
          //s'il n'y pas eu de machines déclarées dans la zone précédemment
          if (!ekf.test(area)){
            //on l'ajoute
            ekf.addMachine(m,area);
            cpt++;
            std::cout << "ajout machine (" << m.x << "," << m.y << ") dans zone " << area << std::endl;
          }
          else {
            int pos = ekf.checkStateVector(m);
            if (pos != 0){
              //std::cout << " position dans le vecteur d'état : " << pos << std::endl;
              //std::cout << "correction machine dans zone " << area << std::endl;
              std::cout << "correction machine à la position " << pos/3 << std::endl;
              ekf.correction(m,pos);
            }
          }
        }*/
      }      
    }

    //std::cout << "machine(s) ajoutée(s) = " << cpt << "\n" << std::endl;

    VectorXd xMean = ekf.getXmean();
    std::cout << "xMean : \n" << xMean << std::endl;

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
    pub_machines.publish(m);
    pub_laser.publish(l);

    m.landmarks.clear();
    l.landmarks.clear();

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}