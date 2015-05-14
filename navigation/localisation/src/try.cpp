#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <random>
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

using namespace Eigen;

int main()
{
  geometry_msgs::Pose2D p;
  Matrix3d m;
  m.setZero();
  Vector3d before, after;

  //translation
  m(1,2) = 0.1;
  
  //rotation
  Matrix2d rot = Rotation2Dd(-M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;
  std::cout << "m : \n" << m << std::endl;

  before(0) = 1;
  before(1) = 1;
  before(2) = 1;
  std::cout << "avant :" << before << std::endl;

  after = m*before;

  std::cout << "après :" << after << std::endl;
}