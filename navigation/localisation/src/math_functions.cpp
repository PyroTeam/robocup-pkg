#include "math_functions.h"

double dist(const geometry_msgs::Point &a, const Line &d)
{
    geometry_msgs::Point b;
    b.x = d.getPoint().x;
    b.y = d.getPoint().y;
    geometry_msgs::Point u;
    u.x = cos(d.getAngle());
    u.y = sin(d.getAngle());
    geometry_msgs::Point ba;
    ba.x = a.x-b.x;
    ba.y = a.y-b.y;

    return std::abs(u.x*ba.y - ba.x*u.y) / sqrt(u.x*u.x + u.y*u.y);
}

double dist(const geometry_msgs::Point &a, const Segment &s)
{
    geometry_msgs::Point b = s.getMin();
    geometry_msgs::Point u;
    u.x = cos(s.getAngle());
    u.y = sin(s.getAngle());
    geometry_msgs::Point ba;
    ba.x = a.x-b.x;
    ba.y = a.y-b.y;

    return std::abs(u.x*ba.y - ba.x*u.y) / sqrt(u.x*u.x + u.y*u.y);
}

double dist(const Segment &seg1, const Segment &seg2)
{
  double distance = geometry_utils::distance(seg1.getCenter(), seg2.getCenter());
  return distance;
}

double angle(const Segment &a, const Segment &b)
{
  return atan((a.getCenter().y-b.getCenter().y)/(a.getCenter().x-b.getCenter().x));
}

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Line &d)
{
    double distance = dist(a,d);
    geometry_msgs::Point v;
    v.x = cos(d.getAngle());
    v.y = sin(d.getAngle());

    double K = ((a.x - d.getPoint().x)*v.x + (a.y - d.getPoint().y)*v.y)/(v.x*v.x + v.y*v.y);

    geometry_msgs::Point p;
    p.x = d.getPoint().x + K*v.x;
    p.y = d.getPoint().y + K*v.y;

    return p;
}

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Segment &s)
{
    double distance = dist(a,s);
    geometry_msgs::Point v;
    v.x = cos(s.getAngle());
    v.y = sin(s.getAngle());

    double K = ((a.x - s.getMin().x)*v.x + (a.y - s.getMin().y)*v.y)/(v.x*v.x + v.y*v.y);

    geometry_msgs::Point p;
    p.x = s.getMin().x + K*v.x;
    p.y = s.getMin().y + K*v.y;

    return p;
}

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const geometry_msgs::Pose2D &p)
{
    double distance = geometry_utils::distance(a,p);
    geometry_msgs::Point v;
    v.x = cos(p.theta);
    v.y = sin(p.theta);

    double K = ((a.x - p.x)*v.x + (a.y - p.y)*v.y)/(v.x*v.x + v.y*v.y);

    geometry_msgs::Point pt;
    pt.x = p.x + K*v.x;
    pt.y = p.y + K*v.y;

    return pt;
}
