#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "conversion_functions.h"
#include "deplacement_msg/Machine.h"

using namespace Eigen;

std::vector<deplacement_msg::Machine> convertIntoMsg(std::vector<Machine> mps)
{
  std::vector<deplacement_msg::Machine> tmp;

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
