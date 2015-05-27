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
  Vector3d before, after;
  Matrix3d m;
  m.setZero();

  before(0) = 2+sqrt(2);
  before(1) = 2+sqrt(2);
  before(2) = 1;

  //translation
  m(0,2) = 0.0;
  m(1,2) = 0.0;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(M_PI/4);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  m.inverse();

  std::cout << m << std::endl;

  after = m*before;

  geometry_msgs::Point p2;
  p2.x     = after(0);
  p2.y     = after(1);

  std::cout << p2 << std::endl;
}