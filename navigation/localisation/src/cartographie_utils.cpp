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
  if ((zone != 0) && (dist(m,getCenter(zone)) <= 0.6))
  {
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

  if (((std::abs(angleA - angleB) <= M_PI/4) ||
       (std::abs(angleA - angleB) >= 3*M_PI/4)) &&
        dist(a,b) <= (a.getSize() + b.getSize())/2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void projection(Segment &worst, Segment &best)
{
  geometry_msgs::Point P1, P2;

  if ((std::abs(best.getAngle())  > M_PI/4 &&
       std::abs(best.getAngle())  < 3*M_PI/4 &&
       std::abs(worst.getAngle()) > M_PI/4 &&
       std::abs(worst.getAngle()) < 3*M_PI/4 &&
       std::abs(best.getAngle() - M_PI_2) <= std::abs(worst.getAngle() - M_PI_2)) ||

      (std::abs(best.getAngle())  < M_PI/4 &&
       std::abs(worst.getAngle()) < M_PI/4 &&
       std::abs(best.getAngle()) <= std::abs(worst.getAngle())) ||

      (std::abs(best.getAngle())  > 3*M_PI/4 &&
       std::abs(worst.getAngle()) > 3*M_PI/4 &&
       std::abs(best.getAngle() - M_PI) <= std::abs(worst.getAngle() - M_PI)) ||

      (std::abs(best.getAngle())  < M_PI/4 &&
       std::abs(worst.getAngle()) > 3*M_PI/4 &&
       std::abs(best.getAngle()) <= std::abs(worst.getAngle() - M_PI)) ||

      (std::abs(best.getAngle())  > 3*M_PI/4 &&
       std::abs(worst.getAngle()) < M_PI/4 &&
       std::abs(best.getAngle() - M_PI) <= std::abs(worst.getAngle())) &&

       best.getSize() >= worst.getSize())
  {
    P1 = ortho(worst.getMin(), best);
    P2 = ortho(worst.getMax(), best);

    worst.setPoints(P1,P2);
    worst.update();
  }
  else 
  {
    P1 = ortho(best.getMin(), worst);
    P2 = ortho(best.getMax(), worst);

    best.setPoints(P1,P2);
    best.update();
  }
}

void modify(Segment a, Segment &b)
{
  projection(a,b);

  //on passe alors dans le repère local du segment pour déterminer s'il y a chevauchement
  geometry_msgs::Point minLocalR = globalToLocal(b.getMin(), b);
  geometry_msgs::Point maxLocalR = globalToLocal(b.getMax(), b);
  geometry_msgs::Point minLocalS = globalToLocal(a.getMin(), b);
  geometry_msgs::Point maxLocalS = globalToLocal(a.getMax(), b);

  Segment tmp = b;

  if(dist(minLocalS,maxLocalS) >= 0.5 &&
     dist(minLocalR,maxLocalR) >= 0.5)
  {
    //si le min du segment vu est avant le min du segment enregistré
    if (minLocalS.x < minLocalR.x/* &&
        maxLocalS.x < maxLocalR.x &&
        std::abs(minLocalS.x - minLocalR.x) <= 0.5*/)
    {
      tmp.setMin(a.getMin());
    }
    //si le max du segment vu est après le max du segment enregistré
    if (maxLocalS.x > maxLocalR.x /*&&
        minLocalS.x > minLocalR.x &&
        std::abs(maxLocalS.x - maxLocalR.x) <= 0.5*/)
    {
      tmp.setMax(a.getMax());
    }
  }

  tmp.update();

  if (tmp.getSize() > b.getSize() &&
      tmp.getSize() > 0.5)
  {
    b = tmp;
  }
}

void adjust(std::list<Segment> &segmentsRecorded, std::list<Segment> segmentsSeen)
{
  bool modified;

  for (auto &sgtS : segmentsSeen)
  {
    modified = false;

    for (auto &sgtR : segmentsRecorded)
    {
      if (isAlmostTheSame(sgtS,sgtR))
      {
        modify(sgtS,sgtR);
        modified = true;
      }
    }
    
    if (!modified)
    {
      segmentsRecorded.push_back(sgtS);
    }
  }
}

std::list<Segment> gather(std::list<Segment> sgts)
{
  std::list<Segment> tmp;

  for (std::list<Segment>::iterator it_1 = sgts.begin(); it_1 != std::prev(sgts.end()); ++it_1)
  {
    Segment segment = *it_1;

    for (std::list<Segment>::iterator it_2 = std::next(it_1,1); it_2 != sgts.end(); ++it_2)
    {
      if (dist(it_1->getMin(), it_2->getMin()) > 0.0 &&
          dist(it_1->getMax(), it_2->getMax()) > 0.0 &&
          isAlmostTheSame(*it_1, *it_2))
      {
        if (dist(it_1->getMax(), it_2->getMin()) < 0.1)
        {
          segment.setMax(it_2->getMax());
          segment.update();
        }
        else if (dist(it_1->getMin(), it_2->getMax()) < 0.1)
        {
          segment.setMin(it_2->getMin());
          segment.update();
        }
      }
    }

    tmp.push_back(segment);
  }

  return tmp;
}

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments)
{
    std::vector<Machine> tmp;

    for (auto &it : listOfSegments)
    {
        if (std::abs(it.getSize() - 0.7) <= 0.05)
        {
            Machine m;
            m.calculateCoordMachine(it);
            tmp.push_back(m);
        }
    }

    return tmp;
}