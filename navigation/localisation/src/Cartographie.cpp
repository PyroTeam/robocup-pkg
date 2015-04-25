#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"
#include "cartographie_utils.h"

#include "EKF_class.h"

using namespace Eigen;

deplacement_msg::Landmarks tabMachines;
deplacement_msg::Landmarks scan;
geometry_msgs::Pose2D      odomRobot;
std::vector<Machine>       mps(24);

void odomCallback(const nav_msgs::Odometry& odom){
  odomRobot.x = odom.pose.pose.position.x;
  odomRobot.y = odom.pose.pose.position.y;
  odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);
}

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser){
  geometry_msgs::Pose2D p;
  Matrix3d m;
  m.setZero();
  Vector3d before;
  Vector3d after;

  //translation
  m(1,2) = 0.1;
  
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  p.x = after(0) ;
  p.y = after(1);
  p.theta = PosLaser.theta;

  return p;
}

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p){
  Vector3d before, after;
  Matrix3d m;
  geometry_msgs::Pose2D p2;
  double angle = odomRobot.theta;

  before(0) = p.x;
  before(1) = p.y;
  before(2) = 1; //toujours 1 ici !

  m.setZero();
  //translation
  m(0,2) = odomRobot.x;
  m(1,2) = odomRobot.y;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(angle - M_PI_2);
  m.topLeftCorner(2,2) = rot;

  m(2,2) = 1;

  //std::cout << m << std::endl;

  after = m*before;

  p2.x   = after(0);
  p2.y   = after(1);
  p2.theta = p.theta;

  return p2;
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  for (auto &it : machines->landmarks){
    geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it));

    int zone = getZone(p);
    if(zone==0)
      continue;

    mps[zone-1].addX(p.x);
    mps[zone-1].addY(p.y);
    mps[zone-1].addTheta(p.theta);
    mps[zone-1].incNbActu();

    mps[zone-1].maj();
  } 
}

void laserCallback(const deplacement_msg::LandmarksConstPtr& laser){
  scan.landmarks.clear();
  for (auto &it : laser->landmarks){
    geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it));
    scan.landmarks.push_back(p);
  }
}    

int main( int argc, char** argv )
{
  ros::init(argc, argv, "EKF_node");

  ros::NodeHandle n;

  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);
  ros::Subscriber sub_machines = n.subscribe("/machines", 1000, machinesCallback);
  ros::Subscriber sub_laser    = n.subscribe("/laser", 1000, laserCallback);

  ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/landmarks", 1000);
  ros::Publisher pub_laser    = n.advertise< deplacement_msg::Landmarks >("/scan_global", 1000);

  ros::Rate loop_rate(5);

  while (n.ok())
  { 
    deplacement_msg::Landmarks tabMPS = convert(mps);

    for (int i = 0; i < tabMPS.landmarks.size(); i++){
      std::cout << "machine (" << tabMPS.landmarks[i].x << "," << tabMPS.landmarks[i].y << ") en zone " << i+1 << std::endl;
    }

    pub_machines.publish(tabMPS);
    pub_laser.publish(scan);

    //tabMachines.landmarks.clear();

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}