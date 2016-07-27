#ifndef _COMMON_UTILS__ZONE__H_
#define _COMMON_UTILS__ZONE__H_


#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"

namespace common_utils {

enum state_t
{
    EMPTY              = 0,
    OCCUPIED           = 1,
    PARTIALLY_OBSERVED = 2,
    UNKNOWN            = 3
};

class Zone
{
protected:
    int m_num;
    int m_state;
    int m_color;
    geometry_msgs::Point m_center;
    double m_width;
    double m_height;
    bool m_in_sight;

public:
    Zone(int number);
    ~Zone();

    int num(){return m_num;}
    int state(){return m_state;}
    int color(){return m_color;}
    double width(){return m_width;}
    double height(){return m_height;}
    bool inSight(){return m_in_sight;}
    geometry_msgs::Point center(){return m_center;}
    bool center(int zone, double &x, double &y);
    // un threshold < 0 signifie une augmentation de la taille initiale de la zone 
    bool isInside(const geometry_msgs::Pose2D &m, float threshold = 0.0);

    void num(int num);
    void state(int state);
    void color(int color);
    void width(double width);
    void height(double height);
    void inSight(bool inSight);
    void center(geometry_msgs::Point center);
    void center(double x, double y);
    void computeCenter();
};

int getArea(const geometry_msgs::Pose2D &m);

std::list<Zone> buildZones();

} // namespace common_utils

#endif // _COMMON_UTILS__ZONE__H_
