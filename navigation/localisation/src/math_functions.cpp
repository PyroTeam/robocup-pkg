#include "math_functions.h"

double dist(geometry_msgs::Point a, Line d)
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

double dist(geometry_msgs::Point a, Segment s)
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

double dist(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

double dist(geometry_msgs::Point a, geometry_msgs::Pose2D b)
{
  return sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m)
{
  return sqrt((m.x - c.x)*(m.x - c.x) + (m.y - c.y)*(m.y - c.y));
}

double dist(Segment seg1, Segment seg2)
{
  double distance = dist(seg1.getCenter(), seg2.getCenter());
  return distance;
}

double angle(Segment a, Segment b)
{
  return atan((a.getCenter().y-b.getCenter().y)/(a.getCenter().x-b.getCenter().x));
}

geometry_msgs::Point ortho(geometry_msgs::Point a, Line d)
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

geometry_msgs::Point ortho(geometry_msgs::Point a, Segment s)
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

geometry_msgs::Point ortho(geometry_msgs::Point a, geometry_msgs::Pose2D p)
{
    double distance = dist(a,p);
    geometry_msgs::Point v;
    v.x = cos(p.theta);
    v.y = sin(p.theta);

    double K = ((a.x - p.x)*v.x + (a.y - p.y)*v.y)/(v.x*v.x + v.y*v.y);

    geometry_msgs::Point pt;
    pt.x = p.x + K*v.x;
    pt.y = p.y + K*v.y;

    return pt;
}

double linReg(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &p)
{

    int          n = points.size();
    double    sumX = 0.0, sumY = 0.0;
    double     ecX = 0.0,  ecY = 0.0;                   //ecart
    double sumEcXY = 0.0;                               //somme des produits des écarts sur x et y
    double    ec2X = 0.0, ec2Y = 0.0;                   //somme des écarts au carré
    double   covXY = 0.0, varX = 0.0, varY = 0.0;

    for(auto &it : points)
    {
        sumX  += it.x;
        sumY  += it.y;
    }

    //calcul des moyennes
    double moyX = sumX/double(n);
    double moyY = sumY/double(n);

    //calcul du coefficient de corrélation
    for(auto &it : points)
    {
        ecX   = it.x - moyX;
        ecY   = it.y - moyY;
        sumEcXY += ecX*ecY;

        ec2X += ecX*ecX;
        ec2Y += ecY*ecY;
    }

    covXY = sumEcXY/double(n);
    varX  = ec2X/double(n);
    varY  = ec2Y/double(n);

    double correl = covXY/sqrt(varX * varY);

    double slope     = covXY/varX;

    p.x = moyX;
    p.y = moyY;
    p.theta = atan2(slope,1);

    return correl*correl;
}