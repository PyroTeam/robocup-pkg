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
			float d = sqrt(msg->markers[i].pose.pose.position.x*msg->markers[i].pose.pose.position.x+msg->markers[i].pose.pose.position.z*msg->markers[i].pose.pose.position.z);
			if(d<2)
			{
				m_foundId = true;
				m_id.push_back(msg->markers[i].id);
				m_positionX.push_back(msg->markers[i].pose.pose.position.x);
				m_positionZ.push_back(msg->markers[i].pose.pose.position.z);
				m_orientationZ.push_back(msg->markers[i].pose.pose.orientation.z);
				m_distance.push_back(d);
			}
		}
		
	}
	if(m_foundId==true)
	{
		ROS_DEBUG("AU MINIMUN UN ARTAG A MOINS DE 2 M");
	}
	else
	{
		ROS_DEBUG("NO ARTAG FOUND");
	}
	
}

ArTagFA::ArTagFA()
{
	m_arTagSub = m_nh.subscribe("ar_pose_marker",1000,&ArTagFA::artagCallback,this);        
}

std::vector<int> ArTagFA::getId()
{
	if(m_id.empty()) 
	{
		ROS_INFO("m_id.size() = %d",(int)m_id.size()); 
		return m_id;
	}
	else
	{
		return m_id;
	}
}

std::vector<float> ArTagFA::getPositionX()
{
	if(m_positionX.empty()) 
	{
		ROS_INFO("m_positionX.size(): %d",(int)m_positionX.size());  
		return m_positionX;
	} 
	else 
	{
		return m_positionX;
	}
}

std::vector<float> ArTagFA::getPositionZ()
{
	if(m_positionZ.empty()) 
	{
		ROS_INFO("m_positionZ.size(): %d",(int)m_positionZ.size()); 
		return m_positionZ;
	}
	else
	{
		return m_positionZ;
	}
}

std::vector<float> ArTagFA::getOrientationZ()
{
	if(m_orientationZ.empty()) 
	{
		ROS_INFO("m_orientationZ.size(): %d",(int)m_orientationZ.size()); 
		return m_orientationZ;
	} 
	else 
	{
		return m_orientationZ;
	}
}

std::vector<float> ArTagFA::getDistance()
{
	if(m_distance.empty()) 
	{
		ROS_INFO("m_distance.size() = %d",(int)m_distance.size()); 
		return m_distance;
	} 
	else 
	{
		return m_distance;
	}
}
