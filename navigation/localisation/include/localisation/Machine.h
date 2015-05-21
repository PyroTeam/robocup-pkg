#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"

class Machine{
public:
	Machine();
	~Machine();

	geometry_msgs::Pose2D getCentre();
	bool getType();

	void setCentre(geometry_msgs::Pose2D c);
	void addX(double x);
	void addY(double y);
	void addTheta(double theta);
	void incNbActu();

	void maj();
	bool exist();

private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	double 					m_thetaSum;
	int 					m_nbActu;
};

#endif