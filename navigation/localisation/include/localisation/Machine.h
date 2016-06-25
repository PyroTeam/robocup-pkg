#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"
#include "comm_msg/ExplorationMachine.h"
#include "Segment.h"

class Machine{
public:
	Machine();
	~Machine();

	geometry_msgs::Pose2D getCentre();
  geometry_msgs::Pose2D reversePose();
  comm_msg::ExplorationMachine msg();

	int getNbActu();
  int color();
	double getReliability();
	double getLastError();

	void setCentre(geometry_msgs::Pose2D c);

  void update(const geometry_msgs::Pose2D &p);
	void calculateCoordMachine(Segment s);
  void color(int color);

  bool canBeUpdated(const geometry_msgs::Pose2D &seenMachine);
  bool neverSeen();
  bool orientationOk();


private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	double 					m_thetaSum;
	int 						m_nbActu;
  int             m_color;
	double					m_reliability;
  double          m_lastError;
  bool            m_orientationOK;
};

#endif
