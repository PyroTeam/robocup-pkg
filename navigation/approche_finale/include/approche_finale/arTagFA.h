#ifndef ARTAG_FA_H
#define ARTAG_FA_H

#include <ros/ros.h>
#include <vector>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class ArTagFA 
{
	public:
		ArTagFA();
		std::vector<int> getId(){ROS_INFO("m_id.size() = %d",(int)m_id.size()); return m_id;}
		std::vector<float> getPositionX(){ROS_INFO("m_positionX.size(): %d",(int)m_positionX.size()); return m_positionX;}
		std::vector<float> getPositionZ(){ROS_INFO("m_positionZ.size(): %d",(int)m_positionZ.size()); return m_positionZ;}
		std::vector<float> getOrientationZ(){ROS_INFO("m_orientationZ.size(): %d",(int)m_orientationZ.size()); return m_orientationZ;}
		std::vector<float> getDistance(){ROS_INFO("m_distance.size() = %d",(int)m_distance.size()); return m_distance;}
		bool getFoundId(){return m_foundId;}
		void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_arTagSub;
		std::vector<int> m_id;
		std::vector<float> m_positionX;
		std::vector<float> m_positionZ;
		std::vector<float> m_orientationZ;
		std::vector<float> m_distance;
		bool m_foundId;

};

#endif
