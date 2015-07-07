#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"

class laserScan
{
public:
	laserScan();
	~laserScan();	

	void PolarToCart();

	float getRangeMin(){
		return m_range_min;
	}
	float getRangeMax(){
		return m_range_max;
	}
	float getAngleMin(){
		return m_angle_min;
	}
	float getAngleMax(){
		return m_angle_max;
	}
	float getAngleInc(){
		return m_angle_inc;
	}
	const std::vector<float>& getRanges() const{
		return m_ranges;
	}
	const std::list<Point>& getPoints() const{
		return m_points;
	}

	void setRangeMin(float min){
		m_range_min=min;
	}	
	void setRangeMax(float max){
		m_range_max=max;
	}
	void setAngleMin(float min){
		m_angle_min=min;
	}
	void setAngleMax(float max){
		m_angle_max=max;
	}	
	void setAngleInc(float inc){
		m_angle_inc=inc;
	}
	void set(const sensor_msgs::LaserScanConstPtr& scan);

	void laserCallback(const sensor_msgs::LaserScanConstPtr& scan); 

private:
	std::vector<float> m_ranges;
	std::list<Point>   m_points;

	float m_range_min;
	float m_range_max;
	float m_angle_min;
	float m_angle_max;
	float m_angle_inc;
public:
	ros::Time m_stamp;
};

#endif