#include "laserScan.h"
#include <cmath>
#include "Point.h"

laserScan::laserScan():m_range_min(0.0),m_range_max(0.0),m_angle_min(0.0),m_angle_max(0.0),m_angle_inc(0.0){

}

laserScan::~laserScan(){

}

void laserScan::PolarToCart (){
	// La marge permet d'exclure assurement les points venant du RangeMax du laser
	static float margin = 0.1;

	for (int i=0; i<m_ranges.size(); i++){
	   	if((m_ranges[i]>getRangeMin()) && (m_ranges[i]<getRangeMax()-margin)){
	   		Point p(m_ranges[i]*cos(getAngleMin() + (float)i*getAngleInc()),
	   				m_ranges[i]*sin(getAngleMin() + (float)i*getAngleInc()));
			m_points.push_back(p);
		}
	}
}

void laserScan::set(const sensor_msgs::LaserScanConstPtr& scan){
	m_points.clear();
	
	setRangeMin(scan->range_min);
	setRangeMax(scan->range_max);
	setAngleMin(scan->angle_min);
	setAngleMax(scan->angle_max);
	setAngleInc(scan->angle_increment);

	m_ranges=scan->ranges;

	PolarToCart();
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
	set(scan);
	m_stamp = scan->header.stamp;
}