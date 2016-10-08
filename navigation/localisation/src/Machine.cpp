#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "Machine.h"

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

double Machine::getReliability()
{
    return m_reliability;
}

void Machine::setCentre(geometry_msgs::Pose2D c)
{
	m_centre = c;
}

void Machine::addX(double x)
{
	m_xSum += x;
}

void Machine::addY(double y)
{
	m_ySum += y;
}

void Machine::addTheta(double theta)
{
	m_thetaSum += theta;
}

void Machine::incNbActu()
{
	m_nbActu++;
}

void Machine::setReliability(double rel)
{
    m_reliability = rel;
}

void Machine::maj()
{
	m_centre.x     = m_xSum/m_nbActu;
	m_centre.y     = m_ySum/m_nbActu;
	m_centre.theta = m_thetaSum/m_nbActu;
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