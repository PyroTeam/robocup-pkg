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
/*
geometry_msgs::Point globalToLocal(const geometry_msgs::Point &p, const Segment &s)
{
  Vector3d before, after;
  Matrix3d m;
  m.setZero();

  before(0) = p.x;
  before(1) = p.y;
  before(2) = 1;

  //translation
  m(0,2) = s.getMin().x;
  m(1,2) = s.getMin().y;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(s.getAngle());
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  after = m.fullPivLu().solve(before);

  geometry_msgs::Point p2;
  p2.x     = after(0);
  p2.y     = after(1);

  return p2;
}
*/

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
/*
std::list<Segment> landmarksToSegments(const deplacement_msg::Landmarks &tabSegments)
{
  std::list<Segment> vect;
  for (int i = 0; i < tabSegments.landmarks.size(); i = i+2)
  {
    Segment s;
    s.setPoints(pose2DToPoint(tabSegments.landmarks[i]),pose2DToPoint(tabSegments.landmarks[i+1]));
    s.update();

    vect.push_back(s);
  }

  return vect;
}

std::vector<geometry_msgs::Pose2D> backToLandmarks(const std::list<Segment> &vect)
{
  std::vector<geometry_msgs::Pose2D> segments;

  for (auto &it : vect)
  {
    segments.push_back(pointToPose2D(it.getMin(), it.getAngle()));
    segments.push_back(pointToPose2D(it.getMax(), it.getAngle()));
  }

  return segments;
}
*/
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
    pose2d.x = point.x;
    pose2d.y = point.y;
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
