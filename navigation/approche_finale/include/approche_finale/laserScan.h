//d apres le code de Thomas Danel

#ifndef laserScan_H
#define laserScan_H

#include <ros/ros.h>
#include <vector>
#include <list>
#include "sensor_msgs/LaserScan.h"
#include "Point.h"
#include "Segment.h"


class laserScan
{
public:

laserScan();
~laserScan();

void Objects();
void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
int max_number_points();
float length(int i);
bool nearby_object(int i);
float distance_objet(int i);
int nearest_object();
Segment build_segment(int i);
float position_y(int i,float d);

float getRangeMin(){return m_range_min;}
float getRangeMax(){return m_range_max;}
float getAngleMin(){return m_angle_min;}
float getAngleMax(){return m_angle_max;}
double getAngleInc(){return m_angle_inc;}
std::vector<float>& getRanges() {return m_ranges;}
std::list<std::vector<Point> >& gettabPoints() {return m_tabpoints;}

void setRangeMin(float min){m_range_min=min;}
void setRangeMax(float max){m_range_max=max;}
void setAngleMin(float min){m_angle_min=min;}
void setAngleMax(float max){m_angle_max=max;}
void setAngleInc(double inc){m_angle_inc=inc;}


private:

std::vector<float> m_ranges;
std::list<std::vector<Point> > m_tabpoints;
float m_range_min;
float m_range_max;
float m_angle_min;
float m_angle_max;
double m_angle_inc;
ros::NodeHandle m_nh;
ros::Subscriber m_ls_sub;

};
#endif
