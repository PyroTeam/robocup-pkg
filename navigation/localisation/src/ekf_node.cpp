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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  EKF ekf = EKF();

  ros::Subscriber sub_odom      = n.subscribe("/new_odom", 1000, &EKF::odomCallback, &ekf);
  ros::Subscriber sub_machines  = n.subscribe("/machines", 1000, &EKF::machinesCallback, &ekf);
  ros::Subscriber sub_laser     = n.subscribe("/laser", 1000, &EKF::laserCallback, &ekf);

  ros::Publisher pub_robot    = n.advertise<geometry_msgs::Point>("/robot", 1000);
  ros::Publisher pub_machines = n.advertise<deplacement_msg::Landmarks>("/landmarks", 1000);
  ros::Publisher pub_laser    = n.advertise<deplacement_msg::Landmarks>("/scan_global", 1000);

  ros::Rate loop_rate(20);

  int cpt = 0;

  while (n.ok())
  {
    if(ekf.initOdom())
    {
      ekf.prediction();
      int area;
  
      //si on observe une machine
      if (ekf.getTabMachines().size() > 0)
      {
        //pour toutes les machines observées
        for (auto &it : ekf.getTabMachines())
        {
          //on regarde si elle appartient à une zone
          int area = ekf.machineToArea(it);
          //si oui
          if (area != 0)
          {
            //si on a déjà vu une machine dans cette zone
            if (ekf.test(area))
            {
              //on corrige sa position et on se recale par rapport à celle ci
              ekf.correction(it, ekf.checkStateVector(it));
            }
            //sinon si elle est assez loin de toutes les machines existantes
            else if (ekf.isFarFromEverything(it))
            {
              // on ajoute cette machine
              std::cout << "ajout machine" << std::endl;
              ekf.addMachine(it);
              cpt++;
            }
          }
        }      
      }
  
      //std::cout << "machine(s) ajoutée(s) = " << cpt << "\n" << std::endl;
  
      VectorXd xMean = ekf.getXmean();
      //std::cout << "xMean : \n" << xMean << std::endl;
  
      ekf.printAreas();
  
      geometry_msgs::Point robot;
      robot.x = xMean(0);
      robot.y = xMean(1);
  
      deplacement_msg::Landmarks m;
      for (int i = 3; i < xMean.rows(); i = i + 3)
      {
        geometry_msgs::Pose2D md;
        md.x     = xMean(i);
        md.y     = xMean(i+1);
        md.theta = xMean(i+2);
        m.landmarks.push_back(md);
      }
  
      deplacement_msg::Landmarks l;
      for (auto &it : ekf.getScan())
      {
        l.landmarks.push_back(it);
      }
  
      pub_robot.publish(robot);
      pub_machines.publish(m);
      pub_laser.publish(l);
  
      m.landmarks.clear();
      l.landmarks.clear();
    }

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}