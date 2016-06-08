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

  void update(const geometry_msgs::Pose2D &p);
	void calculateCoordMachine(Segment s);
  bool canBeUpdated(const geometry_msgs::Pose2D &seenMachine);

private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	double 					m_thetaSum;
	int 						m_nbActu;
	double					m_reliability;
};

#endif
