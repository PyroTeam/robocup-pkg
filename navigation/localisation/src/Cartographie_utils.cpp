#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "cartographie_utils.h"
#include "EKF_class.h"
#include "Machine.h"
#include "Segment.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>


using namespace Eigen;

double dist(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m)
{
  return (m.x - c.x)*(m.x - c.x) + (m.y - c.y)*(m.y - c.y);
}

geometry_msgs::Point middle(Segment s)
{
  geometry_msgs::Point middle;
  middle.x = (s.getMin().x + s.getMax().x)/2;
  middle.y = (s.getMin().y + s.getMax().y)/2;

  return middle;
}

double dist(Segment seg1, Segment seg2)
{
  double distance = dist(middle(seg1), middle(seg2));
  return distance;
}

double angle(Segment a, Segment b)
{
  return atan((middle(a).y-middle(b).y)/(middle(a).x-middle(b).x));
}

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

std::vector<Segment> landmarksToSegments(deplacement_msg::Landmarks tabSegments)
{
  std::vector<Segment> vect;
  for (int i = 0; i < tabSegments.landmarks.size(); i = i+2)
  {
    Segment s;

    geometry_msgs::Point a, b;
    a.x = tabSegments.landmarks[i].x;
    a.y = tabSegments.landmarks[i].y;

    b.x = tabSegments.landmarks[i+1].x;
    b.y = tabSegments.landmarks[i+1].y;

    double size  = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
    double angle =  tan((b.y-a.y)/(b.x-a.x));

    s.setAngle(angle);
    s.setPoints(a,b);
    s.setSize(size);

    vect.push_back(s);
  }

  return vect;
}

bool isAlmostTheSame(Segment a, Segment b)
{
  double angleA = atan2(a.getAngle(),1);
  if (angleA < 0)
  {
    angleA += M_PI;
  }
  double angleB = atan2(b.getAngle(),1);
  if (angleB < 0)
  {
    angleB += M_PI;
  }
  //si l'angle entre les deux est inférieur à 10°
  //et la distance entre les centres est telle qu'il y a chevauchement
  if ((std::abs(angleA - angleB) <= 0.36) &&
      (std::abs(angleA - angleB) <= M_PI - 0.36) &&
       dist(a,b) <= (b.getSize()+a.getSize())/2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void modify(Segment a, Segment &b)
{
  geometry_msgs::Point A, B;

  if(dist(a.getMin(),b) < 0.3 && dist(a.getMax(),b) < 0.3)
  {
    A = ortho(a.getMin(), b);
    B = ortho(a.getMax(), b);

    //on passe alors dans le repère local du segment pour déterminer s'il y a chevauchement
    geometry_msgs::Point minLocalR = globalToLocal(b.getMin(), b);
    geometry_msgs::Point maxLocalR = globalToLocal(b.getMax(), b);
    geometry_msgs::Point minLocalS = globalToLocal(A, b);
    geometry_msgs::Point maxLocalS = globalToLocal(B, b);

    Segment tmp = b;

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

    if (tmp.getSize() > b.getSize() &&
       ((std::abs(tmp.getAngle() - b.getAngle()) <= 0.18) ||
        (std::abs(tmp.getAngle() - b.getAngle()) <= 0.18 + M_PI)))
    {
      tmp.update();
      b = tmp;
    }
  }
}

bool checkAndChangeIfItsNecessary(Segment s, std::vector<Segment> &vect)
{
  for (int i = 0; i < vect.size(); i++)
  {
    if (isAlmostTheSame(s,vect[i]))
    {
      modify(s,vect[i]);
      return true;
    }
  }

  return false;
}

void maj(std::vector<Segment> &segmentsRecorded, std::vector<Segment> segmentsSeen)
{
  for (auto &sgtS : segmentsSeen)
  {
    if (!checkAndChangeIfItsNecessary(sgtS, segmentsRecorded))
    {
      segmentsRecorded.push_back(sgtS);
    }
  }
}

deplacement_msg::Landmarks backToLandmarks(std::vector<Segment> vect)
{
  deplacement_msg::Landmarks segments;

  for (auto &it : vect)
  { 
    geometry_msgs::Pose2D p;
    p.x = it.getMin().x;
    p.y = it.getMin().y;
    p.theta = 0.0;
    segments.landmarks.push_back(p);

    p.x = it.getMax().x;
    p.y = it.getMax().y;
    segments.landmarks.push_back(p);
  }

  return segments;
}