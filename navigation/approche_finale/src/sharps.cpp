#include "sharps.h"
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>

void Sharps::sharpsCallback(const sensor_msgs::PointCloud &msg)
{
	m_obstacle.clear();
	for(int i=0; i<9; i++)
	{
		m_obstacle.push_back(false);
	}
	m_sensorsDistance = msg;
	for(int i=0; i<9; i++)
	{
		float x = (float)m_sensorsDistance.points[i].x;
		float y = (float)m_sensorsDistance.points[i].y;
		//ROS_INFO("distance sensor n %d: x=%f, y=%f,sqrt(x*x+y*y)=%f",i,x,y,sqrt(x*x+y*y)); 
		if(sqrt(x*x+y*y)<0.30)
		{
		m_obstacle[i] = true;
		ROS_INFO("obstacle near distance sensor n %d",i);
		}
	}
}

Sharps::Sharps(){
	m_sharpsSub = m_nh.subscribe("/distance_sensors",1000,&Sharps::sharpsCallback,this);        
}

