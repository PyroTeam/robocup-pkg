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
  int          n = 5;
  double    sumX = 0.0, sumY = 0.0;
  double     ecX = 0.0,  ecY = 0.0;         //ecart
  double sumEcXY = 0.0;                     //somme des produits des écarts sur x et y
  double    ec2X = 0.0, ec2Y = 0.0;         //somme des écarts au carré
  double   covXY = 0.0, varX = 0.0, varY = 0.0;

  std::vector<geometry_msgs::Point> tab;
  //for (int i = 0; i < 5; i++)
  //{
    geometry_msgs::Point p;
    p.x = -4;
    p.y = 4;
    tab.push_back(p);
    p.x = -3;
    p.y = 3;
    tab.push_back(p);
    p.x = -2;
    p.y = 2;
    tab.push_back(p);
    p.x = -1;
    p.y = 1;
    tab.push_back(p);
    p.x = 0;
    p.y = 0;
    tab.push_back(p);
  //}

  for(auto &it : tab)
  {
    sumX  += it.x;
    sumY  += it.y;
  }

  //calcul des moyennes
  double moyX = sumX/double(n);
  double moyY = sumY/double(n);
  std::cout << "point moyen de coord (" << moyX << "," << moyY << ")" << std::endl;

  //calcul du coefficient de corrélation
  for(auto &it : tab)
  {
    ecX   = it.x - moyX;
    ecY   = it.y - moyY;
    sumEcXY += ecX*ecY;

    ec2X += ecX*ecX;
    ec2Y += ecY*ecY;
  }

  covXY = sumEcXY/double(n);
  varX  = ec2X/double(n);
  varY  = ec2Y/double(n);

  double correl = covXY/sqrt(varX * varY);
  std::cout << "corrélation de " << correl*correl << std::endl;
  //correl = correl*correl;

  double slope     = covXY/varX;
  std::cout << "pente de " << slope << std::endl;
  double yIntercept = moyY - slope * moyX;
  std::cout << "ordonnée à l'origine de " << yIntercept << std::endl;
}