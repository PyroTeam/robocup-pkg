#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "conversion_functions.h"

using namespace Eigen;

geometry_msgs::Pose2D changeFrame(const geometry_msgs::Pose2D &p, const tf::StampedTransform &transform)
{
  geometry_msgs::Pose2D result;

  double yaw = tf::getYaw(transform.getRotation());
  result.x     = p.x*cos(yaw) - p.y*sin(yaw) + transform.getOrigin().x();
  result.y     = p.x*sin(yaw) + p.y*cos(yaw) + transform.getOrigin().y();
  result.theta = p.theta + yaw;

  return result;
}

std::vector<geometry_msgs::Pose2D> convert(std::vector<Machine> mps)
{
  std::vector<geometry_msgs::Pose2D> tmp;

  for (auto &it : mps)
  {
    if (it.getNbActu() > 0)
    {
      tmp.push_back(it.getCentre());
    }
  }

  return tmp;
}

geometry_msgs::Pose2D pointToPose2D(const geometry_msgs::Point &point)
{
    geometry_msgs::Pose2D pose2d;
    pose2d.x = point.x;
    pose2d.y = point.y;
    pose2d.theta = 0.0;

    return pose2d;
}

geometry_msgs::Pose2D pointToPose2D(const geometry_msgs::Point &point, double angle)
{
    geometry_msgs::Pose2D pose2d;
    pose2d = pointToPose2D(point);
    pose2d.theta = angle;

    return pose2d;
}

geometry_msgs::Point pose2DToPoint(const geometry_msgs::Pose2D &pose2d)
{
    geometry_msgs::Point point;
    point.x = pose2d.x;
    point.y = pose2d.y;

    return point;
}
