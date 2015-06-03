#include "arTagFA.h"
#include <ros/ros.h>
#include <vector>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

void ArTagFA::artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
	m_foundId=false;
	m_id.clear();
	m_positionX.clear();
	m_positionZ.clear();
	m_orientationZ.clear();
	m_distance.clear();
	if(msg->markers.size()!=0) 
	{
		for(int i=0; i<msg->markers.size(); i++)
		{
			if(sqrt(msg->markers[i].pose.pose.position.x*msg->markers[i].pose.pose.position.x+msg->markers[i].pose.pose.position.z*msg->markers[i].pose.pose.position.z)<2)
			{
				m_foundId = true;
				ROS_INFO("AU MINIMUN UN ARTAG A MOINS DE 2 M");
				m_id.push_back(msg->markers[i].id);
				m_positionX.push_back(msg->markers[i].pose.pose.position.x);
				m_positionZ.push_back(msg->markers[i].pose.pose.position.z);
				m_orientationZ.push_back(msg->markers[i].pose.pose.orientation.z);
				m_distance.push_back(sqrt(m_positionX[i]*m_positionX[i] + m_positionZ[i]*m_positionZ[i]));
			}
		}
		
	}
	else
	{
		ROS_INFO("NO ARTAG FOUND");
	}
	
}

ArTagFA::ArTagFA(){
	m_arTagSub = m_nh.subscribe("/ar_pose_marker",1000,&ArTagFA::artagCallback,this);        
}

