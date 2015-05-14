#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "Machine.h"
#include <cmath>


#include "EKF_class.h"

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
  Matrix2d rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  geometry_msgs::Pose2D p;
  p.x     = after(0) ;
  p.y     = after(1);
  p.theta = PosLaser.theta;

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
  Matrix2d rot = Rotation2Dd(odomRobot.theta - M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  after = m*before;

  geometry_msgs::Pose2D p2;
  p2.x     = after(0);
  p2.y     = after(1);
  p2.theta = p.theta;

  return p2;
}

int getZone(geometry_msgs::Pose2D m)
{
  //côté droit
  if(m.x >= 0 && m.x <= 6 && m.y >= 0 && m.y < 6)
  {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h;
  }
  //côté gauche
  else if (m.x >= -6 && m.x < 0 && m.y >= 0 && m.y < 6)
  {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h + 12;
  }
  else
  {
    return 0;
  }
}

geometry_msgs::Pose2D getCenter(int zone)
{
  geometry_msgs::Pose2D c;

  // Right side
  if(zone<=12)
  {
    c.x = ((zone-1)/4)*2 + 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }
  // Left side
  else if (zone<=24)
  {
    zone -=12;
    c.x = -((zone-1)/4)*2 - 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }

  return c;
}

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m)
{
  return (m.x - c.x)*(m.x - c.x) + (m.y - c.y)*(m.y - c.y);
}

int machineToArea(geometry_msgs::Pose2D m)
{
  int zone = getZone(m);
  if ((zone != 0) && (dist(m,getCenter(zone)) <= 0.36))
  {
    //si on est dans le cercle de centre le centre de zone et de rayon 0.6 m
    //pour éviter un sqrt() on met le seuil au carré
    //std::cout << "machine (" << m.x << "," << m.y << ") en zone " << zone << std::endl;
    return zone;
  }
  else
  {
    return 0;
  }
}

deplacement_msg::Landmarks convert(std::vector<Machine> mps)
{
  deplacement_msg::Landmarks tmp;

  for (auto &it : mps)
  {
    if (it.exist())
    {
      tmp.landmarks.push_back(it.getCentre());
    }
  }

  return tmp;
}