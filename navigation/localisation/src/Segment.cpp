#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "cartographie_utils.h"
#include "math_functions.h"

Segment::Segment() : m_angle(0.0),m_size(0.0)
{

}
Segment::~Segment()
{

}

double Segment::getAngle() const
{
	return m_angle;
}

double Segment::getSize() const
{
	return m_size;
}

geometry_msgs::Point Segment::getMin() const
{
	return m_min;
}

geometry_msgs::Point Segment::getMax() const
{
	return m_max;
}

geometry_msgs::Point Segment::getCenter() const
{
	geometry_msgs::Point middle;
	middle.x = (getMax().x + getMin().x)/2;
	middle.y = (getMax().y + getMin().y)/2;

	return middle;
}

void Segment::setAngle(const double &theta)
{
	m_angle = theta;
	if (m_angle < 0)
	{
		m_angle += M_PI;
	}
}

void Segment::setSize(const double &size)
{
	m_size = size;
}

void Segment::setMin(const geometry_msgs::Point &a)
{
	m_min = a;
}
void Segment::setMax(const geometry_msgs::Point &a)
{
	m_max = a;
}

void Segment::setPoints(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
	m_min = a;
	m_max = b;
}

void Segment::update()
{
	this->setAngle(atan((m_max.y-m_min.y)/(m_max.x-m_min.x)));
	m_size  = geometry_utils::distance(m_min,m_max);
	m_max.z = m_angle;
	m_min.z = m_angle;
}

void Segment::build(const std::list<geometry_msgs::Point> &points){
    geometry_msgs::Pose2D pose2d;

    //on fait une régression linéaire sur le segment
    float correl = linReg(points, pose2d);

    //on projète alors les points extrèmes sur le segment linéarisé
    geometry_msgs::Point ptMin = ortho(points.front(),pose2d);
    geometry_msgs::Point ptMax = ortho(points.back(),pose2d);

    setPoints(ptMin,ptMax);
    update();
/*
    std::cout << "Segment " << std::endl;
    std::cout << " Min(" << getMin().x << ", " << getMin().y << ")" << std::endl;
    std::cout << " Max(" << getMax().x << ", " << getMax().y << ")" << std::endl;
    std::cout << " taille : " << getSize() << std::endl;
    std::cout << " angle  : " << getAngle()*(180/M_PI) << std::endl;
    std::cout << " correlation  : " << correl << std::endl;
*/
}
