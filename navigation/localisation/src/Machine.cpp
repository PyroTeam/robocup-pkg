#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "Machine.h"
#include "math_functions.h"
#include "common_utils/zone.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Machine::Machine() : m_xSum(0.0),m_ySum(0.0),m_thetaSum(0.0),m_nbActu(0.0),m_lastError(10.0),m_reliability(0.0)
{

};

Machine::~Machine()
{

}

geometry_msgs::Pose2D Machine::getCentre()
{
	return m_centre;
}

geometry_msgs::Pose2D Machine::reversePose()
{
  geometry_msgs::Pose2D tmp;
  tmp.x = -m_centre.x;
  tmp.y = m_centre.y;
  tmp.theta = M_PI - m_centre.theta;
  if(tmp.theta > M_PI)
  {
    tmp.theta -= M_PI;
  }
  else if (tmp.theta < -M_PI)
  {
    tmp.theta += M_PI;
  }

  return tmp;
}

comm_msg::ExplorationMachine Machine::msg()
{
  comm_msg::ExplorationMachine tmp;
  tmp.pose = getCentre();
  tmp.zone = common_utils::getArea(tmp.pose);
  tmp.team_color = color();

  return tmp;
}

int Machine::getNbActu()
{
    return m_nbActu;
}

int Machine::color()
{
    return m_color;
}

double Machine::getReliability()
{
  return m_reliability;
}
double Machine::getLastError()
{
  return m_lastError;
}

void Machine::setCentre(geometry_msgs::Pose2D c)
{
	m_centre = c;
}

void Machine::update(const geometry_msgs::Pose2D &p)
{
  m_xSum += p.x;
  m_ySum += p.y;
  m_thetaSum += p.theta;
  m_nbActu++;

  m_lastError = geometry_utils::distance(m_centre, p);

  m_centre.x     = m_xSum/double(m_nbActu);
  m_centre.y     = m_ySum/double(m_nbActu);
  m_centre.theta = m_thetaSum/double(m_nbActu);

  if(m_centre.theta > M_PI)
  {
    m_centre.theta -= M_PI;
  }
  else if (m_centre.theta < -M_PI)
  {
    m_centre.theta += M_PI;
  }
}

void Machine::calculateCoordMachine(Segment s)
{
    double angle     = s.getAngle();
    double absMilieu = (s.getMax().x + s.getMin().x)/2;
    double ordMilieu = (s.getMax().y + s.getMin().y)/2;
    double sinus     = 0.35/2*sin(angle);
    double cosinus   = 0.35/2*cos(angle);

    geometry_msgs::Pose2D center;

    double d1 = (absMilieu - sinus)*(absMilieu - sinus) + (ordMilieu + cosinus)*(ordMilieu + cosinus);
    double d2 = (absMilieu + sinus)*(absMilieu + sinus) + (ordMilieu - cosinus)*(ordMilieu - cosinus);

    if (d1 > d2)
    {
        center.x = absMilieu - sinus;
        center.y = ordMilieu + cosinus;
        center.theta = angle;
    }
    else
    {
        center.x = absMilieu + sinus;
        center.y = ordMilieu - cosinus;
        center.theta = angle;
    }

    if(center.theta > M_PI)
    {
      center.theta -= M_PI;
    }
    else if (center.theta < -M_PI)
    {
      center.theta += M_PI;
    }

    setCentre(center);
}

void Machine::color(int color)
{
  m_color = color;
}

bool Machine::canBeUpdated(const geometry_msgs::Pose2D &seenMachine)
{
  const double delta = M_PI/10;

  if (neverSeen() || std::abs(m_centre.theta - seenMachine.theta) <= delta)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Machine::neverSeen()
{
  return m_nbActu == 0;
}


bool Machine::orientationOk()
{
  return m_orientationOK;
}
