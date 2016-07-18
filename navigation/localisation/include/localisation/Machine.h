#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Machine.h"
#include "Segment.h"

class Machine{
public:
    Machine();
    ~Machine();

    geometry_msgs::Pose2D getCentre();
    geometry_msgs::Pose2D reversePose();
    deplacement_msg::Machine msg();

    int getNbActu();
    int color();
    int zone();
    double getReliability();
    double getLastError();

    void setCentre(geometry_msgs::Pose2D c);
    void setTheta(double theta);

    void update(const geometry_msgs::Pose2D &p);
    void calculateCoordMachine(Segment s);
    void color(int color);
    void zone(int zone);
    void orientation(bool ok);

    bool neverSeen();
    bool orientationOk();
    void switchSides();
    bool isInsideZone(const geometry_msgs::Pose2D &pose, int zone, float zone_width = 2.0, float zone_height = 1.5, float mps_width = 0.35);

    void setId(int idIn)
    {
        m_idIn = idIn;
        m_idOut = idIn+1;
    }


private:
	geometry_msgs::Pose2D 	m_centre;
	double 					m_xSum;
	double 					m_ySum;
	int 						m_nbActu;
  int             m_color;
  int             m_zone;
  double          m_lastError;
  bool            m_orientationOK;
  int             m_idIn;
  int             m_idOut;
};

#endif
