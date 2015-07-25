#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "cartographie_utils.h"
#include "math_functions.h"
#include "EKF_class.h"
#include "Machine.h"
#include "Segment.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;

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

bool isAlmostTheSame(Segment a, Segment b)
{
  double angleA = a.getAngle();
  double angleB = b.getAngle();

  //si l'angle entre les deux est inférieur à 10°
  //et la distance entre les centres est telle qu'il y a chevauchement
  if (((std::abs(angleA - angleB) <= 0.35) ||
       (std::abs(angleA - angleB) <= 0.35+M_PI))&&
       dist(a.getMin(),b) <= 0.2 &&
       dist(a.getMax(),b) <= 0.2)
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
    tmp.setMin(a.getMin());
    tmp.setMax(a.getMax());
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
  else if (std::abs(minLocalS.x - maxLocalR.x) < 0.2)
  {
    tmp.setMin(b.getMin());
    tmp.setMax(B);
    //std::cout << "4" << std::endl;
  }
  else if (std::abs(maxLocalS.x - minLocalR.x) < 0.2)
  {
    tmp.setMin(A);
    tmp.setMax(b.getMax());
    //std::cout << "5" << std::endl;
  }
  
  tmp.update();

  if (tmp.getSize() > b.getSize() &&
      std::abs(tmp.getAngle() - b.getAngle()) <= 0.17)
  {
    b = tmp;
  }
}

void maj(std::vector<Segment> &segmentsRecorded, std::vector<Segment> segmentsSeen)
{
  bool modified;

  for (auto &sgtS : segmentsSeen)
  {
    modified = false;

    for (auto &sgtR : segmentsRecorded)
    {
      if (isAlmostTheSame(sgtS,sgtR))
      {
        //modify(sgtS,sgtR);
        modified = true;
      }
    }
    
    if (!modified)
    {
      segmentsRecorded.push_back(sgtS);
    }
  }
}