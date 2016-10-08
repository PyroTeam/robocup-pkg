#include "cartographie_utils.h"

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

bool isTheSame(Segment a, Segment b)
{
  if (dist(a.getMin(), b.getMin()) == 0 &&
      dist(a.getMax(), b.getMax()) == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isAlmostTheSame(Segment a, Segment b)
{
  double angleA = a.getAngle();
  double angleB = b.getAngle();

  if (((std::abs(angleA - angleB) <= M_PI/4) ||
       (std::abs(angleA - angleB) >= 3*M_PI/4)) &&
        dist(a,b) <= 4.0)
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

bool modify(Segment a, Segment &b)
{
  projection(a,b);

  //on passe alors dans le repère local du segment pour déterminer s'il y a chevauchement
  geometry_msgs::Point bMin = globalToLocal(b.getMin(), b);
  geometry_msgs::Point bMax = globalToLocal(b.getMax(), b);
  geometry_msgs::Point aMin = globalToLocal(a.getMin(), b);
  geometry_msgs::Point aMax = globalToLocal(a.getMax(), b);

  Segment tmp;

  if(dist(aMin,aMax) >= 0.5 &&
     dist(bMin,bMax) >= 0.5 &&
     (dist(aMin, bMin) <= 0.4 || dist(aMin, bMax) <= 0.4) &&
     (dist(aMax, bMax) <= 0.4 || dist(aMax, bMin) <= 0.4))
  {
    if (aMin.x <= bMin.x)
    {
      tmp.setMin(a.getMin());

      if (aMax.x <= bMax.x)
      {
        tmp.setMax(b.getMax());
      }
      else
      {
        tmp.setMax(a.getMax());
      }
    } 
    else    
    {
      tmp.setMin(b.getMin());

      if (aMax.x <= bMax.x)
      {
        tmp.setMax(b.getMax());
      }
      else
      {
        tmp.setMax(a.getMax());
      }
    }
  }

  tmp.update();

  if (tmp.getSize() > b.getSize() &&
      tmp.getSize() > 0.5)
  {
    b = tmp;
    return true;
  }
  else
  {
    return false;
  }
}

void adjust(std::list<Segment> &segmentsRecorded, std::list<Segment> segmentsSeen)
{
  int nb;

  for (auto &sgtS : segmentsSeen)
  {
    nb = 0;

    for (auto &sgtR : segmentsRecorded)
    {
      if (isAlmostTheSame(sgtS,sgtR) && modify(sgtS,sgtR))
      {
        nb++;
      }
    }
    
    if (nb == 0)
    {
      segmentsRecorded.push_back(sgtS);
    }
  }
}

void gatherTwoSegments(Segment &segment_1, Segment segment_2)
{
  if (dist(segment_2.getMax(), segment_1.getMin()) < 1.0)
  {
    segment_1.setMin(segment_2.getMin());
    segment_1.update();
  }
  else if(dist(segment_1.getMax(), segment_2.getMin()) < 1.0)
  {
    segment_1.setMax(segment_2.getMax());
    segment_1.update();
  }
}

void gatherOneSegmentWithAList(Segment &segment, std::list<Segment> &sgts)
{
  std::list<Segment>::iterator it = sgts.begin();

  while (it != sgts.end())
  {
    if(((std::abs(segment.getAngle() - it->getAngle()) <= M_PI/4) ||
        (std::abs(segment.getAngle() - it->getAngle()) >= 3*M_PI/4)) &&
        ((dist(segment.getMax(), it->getMin()) < 0.3) ||
         (dist(it->getMax(), segment.getMin()) < 0.3)) && 
          !isTheSame(segment, *it))
    {
      gatherTwoSegments(segment, *it);
      it = sgts.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void gather(std::list<Segment> &sgts)
{
  std::list<Segment>::iterator it = sgts.begin();
  
  while(it != sgts.end())
  {
    gatherOneSegmentWithAList(*it, sgts);
    ++it;
  }
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