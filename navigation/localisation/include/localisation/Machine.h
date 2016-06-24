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
  int zone();
	double getReliability();
	double getLastError();

	void setCentre(geometry_msgs::Pose2D c);

  void update(const geometry_msgs::Pose2D &p);
	void calculateCoordMachine(Segment s);
  void color(int color);
  void zone(int zone);

  bool neverSeen();
  bool orientationOk();
  void switchSides();


private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	int 						m_nbActu;
  int             m_color;
  int             m_zone;
  double          m_lastError;
  bool            m_orientationOK;
};

#endif
