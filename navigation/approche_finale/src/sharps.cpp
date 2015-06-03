#include "sharps.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

void Sharps::sharpsCallback(const sensor_msgs::PointCloud &msg)
{
	m_sensorsDistance = msg;
	for(int i=0; i<9; i++)
	{
		float x = m_sensorsDistance.points[i].x;
		float y = m_sensorsDistance.points[i].y;
		ROS_INFO("distance sensor n %d: x=%f, y=%f,sqrt(x*x+y*y)=%f",i,x,y,sqrt(x*x+y*y)); 
	}
}

Sharps::Sharps(){
	m_sharpsSub = m_nh.subscribe("/distance_sensors",1000,&Sharps::sharpsCallback,this);        
}

