#include "cartographie_utils.h"

using namespace Eigen;

// Field = 6 m x 12 m
int getArea(geometry_msgs::Pose2D m)
{
  // Right side
  if(m.x >= 0 && m.x <= 6 && m.y >= 0 && m.y < 6)
  {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h;
  }
  // Left Side
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

// Area = 2 m x 1.5 m
geometry_msgs::Pose2D getCenter(int area)
{
  geometry_msgs::Pose2D c;

  // Right side
  if(area<=12)
  {
    c.x = ((area-1)/4)*2 + 1;
    c.y = ((area-1)%4)*1.5 + 0.75;
  }
  // Left side
  else if (area<=24)
  {
    area -=12;
    c.x = -((area-1)/4)*2 - 1;
    c.y = ((area-1)%4)*1.5 + 0.75;
  }

  return c;
}

int machineToArea(geometry_msgs::Pose2D m)
{
  int area = getArea(m);
  if ((area != 0) && (dist(m,getCenter(area)) <= 0.75))
  {
    return area;
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

// The aim of the following function is to determine on which segment we will
// project the other in order to get the best
// speaking of angle, the best will be the closest angle to the horizontal or vertical direction
// speaking of size, the best will be the longer
void projection(Segment &worst, Segment &best)
{
  geometry_msgs::Point P1, P2;

  // The angle is normalised between 0 and M_PI

  // Vertical way
  if ((std::abs(best.getAngle())  > M_PI/4 &&
       std::abs(best.getAngle())  < 3*M_PI/4 &&
       std::abs(worst.getAngle()) > M_PI/4 &&
       std::abs(worst.getAngle()) < 3*M_PI/4 &&
       std::abs(best.getAngle() - M_PI_2) <= std::abs(worst.getAngle() - M_PI_2)) ||

  // Horizontal ways
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

bool improve(Segment a, Segment &b)
{
  // Projection of the worst on the best
  projection(a,b);

  // We transform the coordinates from global to local frame of the first segment
  geometry_msgs::Point bMin = globalToLocal(b.getMin(), b);
  geometry_msgs::Point bMax = globalToLocal(b.getMax(), b);
  geometry_msgs::Point aMin = globalToLocal(a.getMin(), b);
  geometry_msgs::Point aMax = globalToLocal(a.getMax(), b);

  Segment tmp;
  const double threshold = 0.3;

  // 0.5 is the minimal size for a well detected segment
  if(dist(aMin,aMax) >= 0.5 && dist(bMin,bMax) >= 0.5 &&
  // if there is an overlap between the two segments
     (dist(aMin, bMin) <= threshold || dist(aMin, bMax) <= threshold) &&
     (dist(aMax, bMax) <= threshold || dist(aMax, bMin) <= threshold))
  {
    if (aMin.x <= bMin.x)
    {
      if (aMax.x <= bMax.x)
      {
      //            A    <------------------------>
      //            B        <----------------------->
      //            tmp  <--------------------------->
        tmp.setMin(a.getMin());
        tmp.setMax(b.getMax());
        tmp.update();
      }
      else
      {
      //            A    <--------------------------->
      //            B        <--------------------->
      //            tmp  <--------------------------->
        tmp = a;
      }
    }
    else
    {
      if (aMax.x <= bMax.x)
      {
      //            A        <--------------------->
      //            B    <--------------------------->
      //            tmp  <--------------------------->
        tmp = b;
      }
      else
      {
      //            A        <----------------------->
      //            B    <------------------------->
      //            tmp  <--------------------------->
        tmp.setMin(b.getMin());
        tmp.setMax(a.getMax());
        tmp.update();
      }
    }
  }

  // if we improved that segment
  if (tmp.getSize() > b.getSize())
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
      if (isAlmostTheSame(sgtS,sgtR) && improve(sgtS,sgtR))
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
  if (dist(segment_2.getMax(), segment_1.getMin()) < 0.5)
  {
    segment_1.setMin(segment_2.getMin());
    segment_1.update();
  }
  else if(dist(segment_1.getMax(), segment_2.getMin()) < 0.5)
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
