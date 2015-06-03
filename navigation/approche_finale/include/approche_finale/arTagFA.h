#ifndef ARTAG_FA_H
#define ARTAG_FA_H

#include <ros/ros.h>
#include <vector>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class ArTagFA 
{
	public:
		ArTagFA();
		std::vector<int> getId(){return m_id;}
		std::vector<float> getPositionX(){return m_positionX;}
		std::vector<float> getPositionZ(){return m_positionZ;}
		std::vector<float> getOrientationZ(){return m_orientationZ;}
		std::vector<float> getDistance(){return m_distance;}
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
