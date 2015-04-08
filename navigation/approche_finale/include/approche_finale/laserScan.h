//d apres le code de Thomas Danel

#ifndef laserScan_H
#define laserScan_H
#include <vector>
#include <list>
#include "sensor_msgs/LaserScan.h"
#include "Point.h"
class laserScan
{
public:

laserScan();
~laserScan();

void Objects();
void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
int max_number_points();
bool around_70cm(int i);
bool nearby_object(int i);

float getRangeMin(){return m_range_min;}
float getRangeMax(){return m_range_max;}
float getAngleMin(){return m_angle_min;}
float getAngleMax(){return m_angle_max;}
float getAngleInc(){return m_angle_inc;}
const std::vector<float>& getRanges() const{return m_ranges;}
const std::list<std::vector<Point> >& gettabPoints() const{return m_tabpoints;}

void setRangeMin(float min){m_range_min=min;}
void setRangeMax(float max){m_range_max=max;}
void setAngleMin(float min){m_angle_min=min;}
void setAngleMax(float max){m_angle_max=max;}
void setAngleInc(float inc){m_angle_inc=inc;}


private:

std::vector<float> m_ranges;
std::list<std::vector<Point> > m_tabpoints;
float m_range_min;
float m_range_max;
float m_angle_min;
float m_angle_max;
float m_angle_inc;

};
#endif
