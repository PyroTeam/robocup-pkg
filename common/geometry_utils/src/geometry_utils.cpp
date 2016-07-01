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
  result.theta = p.theta + yaw;

  return result;
}

} // namespace geometry_utils
