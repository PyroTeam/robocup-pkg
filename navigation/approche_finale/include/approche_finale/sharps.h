#ifndef SHARPS_H
#define SHARPS_H

#include <ros/ros.h>
#include <vector>
#include "sensor_msgs/PointCloud.h"

class Sharps 
{
	public:
		Sharps();
		sensor_msgs::PointCloud getSensorsDistance(){return m_sensorsDistance;}
		std::vector<bool> getObstacle(){return m_obstacle;}
		void sharpsCallback(const sensor_msgs::PointCloud &msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_sharpsSub;
		sensor_msgs::PointCloud m_sensorsDistance;
		std::vector<bool> m_obstacle;

};

#endif
