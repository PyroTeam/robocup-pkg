#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"
#include "common_utils/Zone.h"
#include "common_utils/MPS.h"
#include "deplacement_msg/Machine.h"
#include "Segment.h"

using namespace common_utils;

class Machine : public MPS
{
public:
    Machine();
    ~Machine();

    geometry_msgs::Pose2D reversePose();
    deplacement_msg::Machine msg();

    int getNbActu();
    double getReliability();
    double getLastError();
    double theta(){return pose().theta;}

    void theta(double theta);
    void update(const geometry_msgs::Pose2D &p);
    void calculateCoordMachine(Segment s);
    void switchSides();

    bool neverSeen();
    bool isInsideZone(int zone);

private:
    double m_xSum;
    double m_ySum;
    int    m_nbActu;
    double m_lastError;
};

#endif
