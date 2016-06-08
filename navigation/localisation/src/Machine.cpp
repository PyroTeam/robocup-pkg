#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "Machine.h"
#include "math_functions.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Machine::Machine() : m_xSum(0.0),m_ySum(0.0),m_thetaSum(0.0),m_nbActu(0.0)
{

};

Machine::~Machine()
{

}

geometry_msgs::Pose2D Machine::getCentre()
{
	return m_centre;
}

int Machine::getNbActu()
{
    return m_nbActu;
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

  m_centre.x     = m_xSum/double(m_nbActu);
  m_centre.y     = m_ySum/double(m_nbActu);
  m_centre.theta = m_thetaSum/double(m_nbActu);
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
    setCentre(center);
}


bool Machine::canBeUpdated(const geometry_msgs::Pose2D &seenMachine)
{
  const double threshold = 0.3;

  if (m_nbActu == 0 || (dist(m_centre, seenMachine) <= threshold && std::abs(m_centre.theta - seenMachine.theta) <= M_PI/10))
  {
    return true;
  }
  else
  {
    return false;
  }
}
