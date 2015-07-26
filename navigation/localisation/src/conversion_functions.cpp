#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "conversion_functions.h"

using namespace Eigen;

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser)
{
  //matrice de transformation
  Matrix3d m;
  m.setZero();

  Vector3d before, after;

  //translation
  m(1,2) = 0.1;
  
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  geometry_msgs::Pose2D p;
  p.x     = after(0) ;
  p.y     = after(1);
  p.theta = PosLaser.theta + M_PI_2;

  return p;
}

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, geometry_msgs::Pose2D odomRobot)
{
  Vector3d before, after;
  Matrix3d m;
  m.setZero();

  before(0) = p.x;
  before(1) = p.y;
  before(2) = 1;

  //translation
  m(0,2) = odomRobot.x;
  m(1,2) = odomRobot.y;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(odomRobot.theta - M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  after = m*before;

  geometry_msgs::Pose2D p2;
  p2.x     = after(0);
  p2.y     = after(1);
  p2.theta = p.theta - M_PI_2;

  return p2;
}

geometry_msgs::Point globalToLocal(geometry_msgs::Point p, Segment s)
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


deplacement_msg::Landmarks convert(std::vector<Machine> mps)
{
  deplacement_msg::Landmarks tmp;

  for (auto &it : mps)
  {
    if (it.getNbActu() > 0)
    {
      tmp.landmarks.push_back(it.getCentre());
    }
  }

  return tmp;
}

std::list<Segment> landmarksToSegments(deplacement_msg::Landmarks tabSegments)
{
  std::list<Segment> vect;
  for (int i = 0; i < tabSegments.landmarks.size(); i = i+2)
  {
    Segment s;

    geometry_msgs::Point a, b;
    a.x = tabSegments.landmarks[i].x;
    a.y = tabSegments.landmarks[i].y;

    b.x = tabSegments.landmarks[i+1].x;
    b.y = tabSegments.landmarks[i+1].y;

    s.setPoints(a,b);
    s.update();

    vect.push_back(s);
  }

  return vect;
}

deplacement_msg::Landmarks backToLandmarks(std::list<Segment> vect)
{
  deplacement_msg::Landmarks segments;

  for (auto &it : vect)
  { 
    geometry_msgs::Pose2D p;
    p.x = it.getMin().x;
    p.y = it.getMin().y;
    p.theta = it.getAngle();
    segments.landmarks.push_back(p);

    p.x = it.getMax().x;
    p.y = it.getMax().y;
    segments.landmarks.push_back(p);
  }

  return segments;
}

geometry_msgs::Pose2D pointToPose2D(geometry_msgs::Point point)
{
    geometry_msgs::Pose2D pose2d;
    pose2d.x = point.x;
    pose2d.y = point.y;
    pose2d.theta = 0.0;

    return pose2d;
}

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose2d)
{
    geometry_msgs::Point point;
    point.x = pose2d.x;
    point.y = pose2d.y;

    return point;
}