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

void Machine::maj()
{
	m_centre.x     = m_xSum/m_nbActu;
	m_centre.y     = m_ySum/m_nbActu;
	m_centre.theta = m_thetaSum/m_nbActu;
}

bool Machine::exist()
{
	if (m_nbActu >= 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Machine::calculateCoordMachine(Segment s)
{
    double angle = s.getAngle();
    double absMilieu = (s.getMax().x + s.getMin().x)/2;
    double ordMilieu = (s.getMax().y + s.getMin().y)/2;

    geometry_msgs::Pose2D center;

    //on met l'angle entre -M_PI_2 et M_PI_2
    angle = atan(tan(angle));
    //puis entre 0 et M_PI 
    //if (angle < 0)
    //{
    //    angle += M_PI;
    //}
    center.theta = angle;

    //si l'angle est > 0
    if (angle > 0.0)
    {
        center.x = absMilieu + 0.35/2*sin(angle);
        center.y = ordMilieu - 0.35/2*cos(angle);
    }
    //si l'angle est <= 0
    else 
    {
        center.x = absMilieu - 0.35/2*sin(angle);
        center.y = ordMilieu + 0.35/2*cos(angle);
    }

    setCentre(center);
}