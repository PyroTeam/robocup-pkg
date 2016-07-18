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

Machine::Machine() : m_xSum(0.0),m_ySum(0.0),m_nbActu(0.0), m_color(2), m_zone(0), m_lastError(std::numeric_limits<int>::max()), m_orientationOK(false)
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
  tmp.theta = geometry_utils::normalizeAngle(M_PI - m_centre.theta);

  return tmp;
}

deplacement_msg::Machine Machine::msg()
{
  deplacement_msg::Machine tmp;
  tmp.pose = m_centre;
  tmp.zone = m_zone;
  tmp.orientationOk = m_orientationOK;
  tmp.idIn = m_idIn;
  tmp.idOut = m_idOut;
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

int Machine::zone()
{
    return m_zone;
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
  if (neverSeen())
  {
    m_centre.x     = p.x;
    m_centre.y     = p.y;
    m_centre.theta = geometry_utils::normalizeAngle(p.theta);

    m_xSum += p.x;
    m_ySum += p.y;
    m_nbActu++;
  }
  else if (m_nbActu < 10)
  {
    m_xSum += p.x;
    m_ySum += p.y;
    m_nbActu++;

    m_centre.x     = m_xSum/double(m_nbActu);
    m_centre.y     = m_ySum/double(m_nbActu);

    double tmp = geometry_utils::normalizeAngle(p.theta);
    double diff = std::abs(m_centre.theta - tmp);
    // Si l'écart est grand cela signifie que la machine est vue avec le mauvais angle par rapport à celui enregistré
    // cela est dû à la transformation TF du repère tower_camera_link vers map
    if (diff > M_PI_2)
    {
      if (tmp > 0)
      {
        tmp -= M_PI;
      }
      else
      {
        tmp += M_PI;
      }
    }
    m_centre.theta = geometry_utils::normalizeAngle((m_centre.theta + tmp)/2);

    m_lastError = geometry_utils::distance(m_centre, p) + std::abs(diff);
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

    setCentre(center);
}

void Machine::color(int color)
{
  m_color = color;
}

void Machine::zone(int zone)
{
  m_zone = zone;
}

void Machine::orientation(bool ok)
{
  m_orientationOK = ok;
}

bool Machine::neverSeen()
{
  return m_nbActu == 0;
}

bool Machine::orientationOk()
{
  return m_orientationOK;
}

void Machine::switchSides()
{
  if (m_centre.theta < 0)
  {
    m_centre.theta += M_PI;
  }
  else
  {
    m_centre.theta -= M_PI;
  }

  m_orientationOK = true;
}

bool Machine::isInsideZone(const geometry_msgs::Pose2D &pose, int zone, float zone_width, float zone_height, float mps_width)
{
  geometry_msgs::Pose2D center;
  common_utils::getZoneCenter(zone, center.x, center.y);

  // si la machine est dans le rectangle inscrit à la zone
  if ((std::abs(center.x - pose.x) <= (zone_width-mps_width)/2) &&
      (std::abs(center.y - pose.y) <= (zone_height-mps_width)/2))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Machine::setTheta(double theta)
{
  m_centre.theta = theta;
}
