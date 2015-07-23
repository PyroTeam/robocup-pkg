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
#include "math_functions.h"
#include "Segment.h"
#include "cartographie_utils.h"
#include "conversion_functions.h"

using namespace Eigen;

int main()
{
  Segment a,b;
  // b segment enregistré
  // a segment vu
  geometry_msgs::Point minA, maxA, minB, maxB;

  maxB.x = -6.04102832559;
  maxB.y = 1.00406327055;
  minB.x = -6.05092146873;
  minB.y = 0.298387377851;

  maxA.x = -6.04333610333;
  maxA.y = 1.02160698346;
  minA.x = -6.03395528922;
  minA.y = 0.022179961326;

  a.setPoints(minA, maxA);
  a.update();
  b.setPoints(minB, maxB);
  b.update();

  std::cout << a.getAngle() << std::endl;
  std::cout << b.getAngle() << std::endl;

  double angleA = a.getAngle();
  double angleB = b.getAngle();

  //si l'angle entre les deux est inférieur à 20°
  //et la distance entre les centres est telle qu'il y a chevauchement
  if (((std::abs(angleA - angleB) <= 0.35) ||
       (std::abs(angleA - angleB) <= 0.35+M_PI))&&
       dist(a,b) <= (b.getSize()+a.getSize())/2)
  {
    printf("almost the same\n");
  }
  else
  {
    printf("not the same\n");
  }
/*
  std::cout << "Segment a" << std::endl;
  std::cout << "(" << a.getMin() << "," << a.getMax() << ")" << std::endl;
  std::cout << "Segment b" << std::endl;
  std::cout << "(" << b.getMin() << "," << b.getMax() << ")" << std::endl;
*/
  geometry_msgs::Point A, B;

  std::cout << "OK" << std::endl;

  A = ortho(a.getMin(), b);
  B = ortho(a.getMax(), b);

  //on passe alors dans le repère local du segment pour déterminer s'il y a chevauchement
  geometry_msgs::Point minLocalR = globalToLocal(b.getMin(), b);
  geometry_msgs::Point maxLocalR = globalToLocal(b.getMax(), b);
  geometry_msgs::Point minLocalS = globalToLocal(A, b);
  geometry_msgs::Point maxLocalS = globalToLocal(B, b);

  std::cout << "(" << minLocalR.x << "," << minLocalR.y << ")" << std::endl;
  std::cout << "(" << maxLocalR.x << "," << maxLocalR.y << ")" << std::endl;
  std::cout << "(" << minLocalS.x << "," << minLocalS.y << ")" << std::endl;
  std::cout << "(" << maxLocalS.x << "," << maxLocalS.y << ")" << std::endl;

  Segment tmp;

  //si le segment enregistré est inclu dans le segment vu
  if (minLocalS.x < minLocalR.x && maxLocalS.x > maxLocalR.x)
  {
    tmp.setMin(A);
    tmp.setMax(B);
    //std::cout << "1" << std::endl;
  }
  //si le min du segment vu est avant le min du segment enregistré
  else if (minLocalS.x < minLocalR.x && maxLocalS.x < maxLocalR.x)
  {
    tmp.setMin(A);
    //std::cout << "2" << std::endl;
  }
  //si le max du segment vu est après le max du segment enregistré
  else if (minLocalS.x > minLocalR.x && maxLocalS.x > maxLocalR.x)
  {
    tmp.setMax(B);
    //std::cout << "3" << std::endl;
  }
  else if (std::abs(minLocalS.x - maxLocalR.x) < 0.3)
  {
    tmp.setMin(b.getMin());
    tmp.setMax(B);
    //std::cout << "4" << std::endl;
  }
  else if (std::abs(maxLocalS.x - minLocalR.x) < 0.3)
  {
    tmp.setMin(A);
    tmp.setMax(b.getMax());
    //std::cout << "5" << std::endl;
  }

  std::cout << "Segment a" << std::endl;
  std::cout << "(" << a.getMin() << "," << a.getMax() << ")" << std::endl;

  std::cout << "Segment b" << std::endl;
  std::cout << "(" << b.getMin() << "," << b.getMax() << ")" << std::endl;

  std::cout << "Segment tmp before" << std::endl;
  std::cout << "(" << tmp.getMin() << "," << tmp.getMax() << ")" << std::endl;
  
  std::cout << tmp.getAngle() << std::endl;
  tmp.update();
  std::cout << tmp.getAngle() << std::endl;

  std::cout << "Segment tmp after" << std::endl;
  std::cout << "(" << tmp.getMin() << "," << tmp.getMax() << ")" << std::endl;
}