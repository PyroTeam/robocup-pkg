#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

#include "EKF_class.h"

using namespace Eigen;

deplacement_msg::Landmarks tabMachines;
deplacement_msg::Landmarks scan;
geometry_msgs::Pose2D      odomRobot;
geometry_msgs::Pose2D      initRobot;

int machineToArea(geometry_msgs::Pose2D m){
  int zone = 0;

  if(m.x >= 0 && m.y >= 0) {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    zone = w*4 + h;
  }
  else if (m.x < 0 && m.y >= 0) {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    zone = w*4 + h + 12;
  }
  else {
    int zone = 0;
  }

  return zone;
}

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
  //correctAngle(angle);

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

  after = m*before;

  p2.x   = after(0);
  p2.y   = after(1);
  p2.theta = p.theta/* - angle*/;

  return p2;
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  for (auto &it : machines->landmarks){
    geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it));

    //std::cout << "machine en zone " << machineToArea(p) << std::endl;
    if (machineToArea(p) >= 12){
        tabMachines.landmarks.push_back(p);
    }
  }
}

void laserCallback(const deplacement_msg::LandmarksConstPtr& laser){
  scan.landmarks.clear();
  for (auto &it : laser->landmarks){
    geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it));
    scan.landmarks.push_back(p);
  }
}    

void init(std::string s, int n){
  /*
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
  }*/
  initRobot.x     = 0.0;
  initRobot.y     = 0.0;
  initRobot.theta = 0.0;
}

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
 
  //std::cout << s << "\n" << num << std::endl;

  init(s, num);

  ros::Subscriber sub_odom     = n.subscribe("/new_odom", 1000, odomCallback);
  ros::Subscriber sub_machines = n.subscribe("/machines", 1000, machinesCallback);
  ros::Subscriber sub_laser    = n.subscribe("/laser", 1000, laserCallback);

  //transmettre le nom du topic à Valentin !!!
  ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/landmarks", 1000);
  ros::Publisher pub_laser    = n.advertise< deplacement_msg::Landmarks >("/scan_global", 1000);

  ros::Rate loop_rate(5);

  while (n.ok())
  { 
    pub_machines.publish(tabMachines);
    pub_laser.publish(scan);

    tabMachines.landmarks.clear();

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}