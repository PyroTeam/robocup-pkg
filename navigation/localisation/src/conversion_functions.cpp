#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "conversion_functions.h"
#include "comm_msg/ExplorationMachine.h"

using namespace Eigen;

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

std::vector<comm_msg::ExplorationMachine> convertIntoMsg(std::vector<Machine> mps)
{
  std::vector<comm_msg::ExplorationMachine> tmp;

  for (int i = 0; i < mps.size(); i++)
  {
    if (!mps[i].neverSeen())
    {
      tmp.push_back(mps[i].msg());
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
