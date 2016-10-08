#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"
#include "Segment.h"

class Machine{
public:
	Machine();
	~Machine();

	geometry_msgs::Pose2D getCentre();
	int getNbActu();
	double getReliability();

	void setCentre(geometry_msgs::Pose2D c);
	void addX(double x);
	void addY(double y);
	void addTheta(double theta);
	void incNbActu();
	void setReliability(double rel);

	void maj();
	void calculateCoordMachine(Segment s);
private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	double 					m_thetaSum;
	int 					m_nbActu;
	double					m_reliability;
};

#endif