/**
 * \file        geometry_utils.cpp
 *
 * \brief       bibliothèque de fonctions de calculs géometriques
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-23
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "geometry_utils.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace geometry_utils {

double distance(const geometry_msgs::Point &p0, const nav_msgs::Path &path)
{
    // TODO

    return 0.0;
}

geometry_msgs::Point midPoint(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
{
    geometry_msgs::Point m;
    m.x = (p0.x+p1.x)/2.0;
    m.y = (p0.y+p1.y)/2.0;
    m.z = (p0.z+p1.z)/2.0;
    return m;
}


geometry_msgs::Pose2D changeFrame(const geometry_msgs::Pose2D &p, const tf::StampedTransform &transform)
{
  geometry_msgs::Pose2D result;

  double yaw = tf::getYaw(transform.getRotation());
  result.x     = p.x*cos(yaw) - p.y*sin(yaw) + transform.getOrigin().x();
  result.y     = p.x*sin(yaw) + p.y*cos(yaw) + transform.getOrigin().y();
  result.theta = normalizeAngle(p.theta + yaw);

  return result;
}

geometry_msgs::Pose2D machineToMapFrame(const geometry_msgs::Pose2D &p, const geometry_msgs::Pose2D &poseMachine)
{
  geometry_msgs::Pose2D result;

  double yaw = poseMachine.theta;
  result.x     = p.x*cos(yaw) - p.y*sin(yaw) + poseMachine.x;
  result.y     = p.x*sin(yaw) + p.y*cos(yaw) + poseMachine.y;
  result.theta = normalizeAngle(p.theta + yaw);

  return result;
}


double linearRegression(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &model)
{
  int          n = points.size();
  double    sumX = 0.0, sumY = 0.0;
  double     ecX = 0.0,  ecY = 0.0;                   //ecart
  double sumEcXY = 0.0;                               //somme des produits des écarts sur x et y
  double    ec2X = 0.0, ec2Y = 0.0;                   //somme des écarts au carré
  double   covXY = 0.0, varX = 0.0, varY = 0.0;

  for(auto &it : points)
  {
      sumX  += it.x;
      sumY  += it.y;
  }

  //calcul des moyennes
  double moyX = sumX/double(n);
  double moyY = sumY/double(n);

  //calcul du coefficient de corrélation
  for(auto &it : points)
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

  double slope     = covXY/varX;

  model.x = moyX;
  model.y = moyY;
  model.theta = atan2(slope,1);

  return correl*correl;
}

double linearRegression(const std::vector<geometry_msgs::Point> &points, geometry_msgs::Pose2D &model)
{
  std::list<geometry_msgs::Point> list;
  for (auto &it : points)
  {
    list.push_back(it);
  }

  return linearRegression(list, model);
}

double linearRegression(const std::vector<geometry_msgs::Point> &points, geometry_msgs::Point &pt, double &angle)
{
  geometry_msgs::Pose2D model;
  double correl = linearRegression(points, model);

  pt.x = model.x;
  pt.y = model.y;
  angle = model.theta;

  return correl;
}


} // namespace geometry_utils
