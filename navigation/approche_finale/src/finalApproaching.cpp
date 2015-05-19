#include "finalApproaching.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <manager_msg/finalApproachingAction.h>
#include <cmath>
#include <vector>
#include <list>
#include "bumperlistener.h"
#include "geometry_msgs/Twist.h"
#include "fa_utils.h"
#include "laserScan.h"
#include "Point.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int a=0,b=0,c=0,d=0;

finalApproaching::~finalApproaching(void){}

void finalApproaching::executeCB(const manager_msg::finalApproachingGoalConstPtr &goal)
{
	// helper variables
	ros::Rate loopRate(100);
	m_pubMvt = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	//m_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1000);
	bool success = true;
	BumperListener bp;

	feedback.percent_complete = 0;
	// check that preempt has not been requested by the client
	if (as.isPreemptRequested() || !ros::ok())
	{
		ROS_INFO("%s: Preempted", actionName.c_str());
		// set the action state to preempted
		as.setPreempted();
		success = false;
		//break;
	}
	// publish info to the console for the user
	ROS_INFO("%s: Executing, creating finalApproaching sequence of type %i with side %i and parameter %i", actionName.c_str(), goal->type, goal->side,goal->parameter);
	m_type = goal->type;
	m_side = goal->side;
	m_parameter = goal->parameter;
	   		  
	laserScan ls;
	//loopRate.sleep();
	float positionY;
	float gradient;
	float ortho;
	float moyY;
	float moyO;
	int maxPoints=0;
	int nearby=0;
	int j = 0;
	std::list<float> listPositionY;
	std::list<float> listOrtho;
	/*visualization::Marker msg_marker;
	msg_marker.header.frame.id = "/laser_link";
	msg_marker.header.stamp = ros::Time::now();
	msg_marker.ns = "visualization_segments";
	msg_marker.action = visualization_msgs::Marjer::ADD;
	msg_marker.pose.orientation.w = 1.0;
	msg_marker.id = 2;
	msg_marker.type = visualization_msgs::Marker::LINE_LIST;

	msg_marker.scale.x = 0.1;
	msg_marker.color.r = 1.0;
	msg_marker.color.a = 1.0;
	geometry_msgs::Point p;
	msg_marker.points.push_back(p);
	msg_marker.points.push_back(p);*/
	ROS_INFO("Waiting a complete laserscan");
	while(ros::ok() && !bp.getState() && c!=2)
	{
		if(ls.getRanges().size() == 513 && ls.getTabSegments().size()>0)
		{
			nearby = ls.nearestSegment();
			ROS_INFO(" numero du segment le plus proche = %d",nearby);
			/*msg_marker.points[0].x;
			msg_marker.points[0].y;
			msg_marker.points[1].x;
			msg_marker.points[1].y;
			m_marker_pub.publish(msg_marker);*/
			std::vector<Segment> tabSeg = ls.getTabSegments();
			gradient = tabSeg[nearby].getGradient();
			ROS_INFO("gradient du segment le plus proche : %f",gradient);
			positionY = ls.positionY(tabSeg[nearby]);
			ROS_INFO("l objet se trouve a environ %f m du bord",positionY);
			if(j<10)
			{
				listPositionY.push_back(positionY);
				listOrtho.push_back(ortho);
				j++;
			}
			else
			{
				/*listPositionY.pop_front();
				listOrtho.pop_front();
				listPositionY.push_back(positionY);
				listOrtho.push_back(ortho);*/
				moyY = moy(listPositionY);
				moyO = moy(listOrtho);
				listPositionY.clear();
				listOrtho.clear();
				j=0;
				a = asservissementAngle(m_pubMvt,gradient);
				ortho = ls.distanceOrtho(tabSeg[nearby]);
				/*while(a!=1)
				{
				a = asservissementAngle(m_pubMvt,gradient);
				}
				ortho = ls.distanceOrtho(tabSeg[nearby]);
				while(b!=1)
				{
					if(a==1)
					{
						b = asservissementPositionY(m_pubMvt,positionY,objectifY(),ortho);
					}
				}
				while(c!=2)
				{
					if(a==1 && b==1)
					{
						c=asservissementPositionX(m_pubMvt,ortho,objectifX());
					}
				}*/
				if(a == 1)
				{
					b = asservissementPositionY(m_pubMvt,moyY,objectifY(),moyO);
					if(b == 1)
					{
						c = asservissementPositionX(m_pubMvt,moyO,0.35);	
					}
				}
				/*if(c==2)
				{
					d = asservissementPositionX(m_pubMvt,moyO,objectifX());*/
			}	
			feedback.percent_complete = avancement(a,b,c);
			as.publishFeedback(feedback);
			ros::spinOnce();
			ROS_INFO("etat du bumper: %d ",bp.getState());
			loopRate.sleep();
		}
	}
	if(bp.getState())
	{
		success=false;
		geometry_msgs::Twist stop;
		stop.linear.x = 0;
		stop.linear.y = 0;
		stop.angular.z = 0;
		m_pubMvt.publish(stop);
	}
	if(success)
	{
		result.success = true;
		ROS_INFO("%s: Succeeded", actionName.c_str());
		// set the action state to succeeded
		a=0;
		b=0;
		c=0;
		as.setSucceeded(result);
	}
}


int finalApproaching::avancement(int a, int b, int c)
{
	int tmp=-20;
	if(c==2)
	{
		tmp = 100;
	}
	else
	{
		if(b == 1 && (c == 0 || c==1))
		{
			tmp = 67;
		}	
		else
		{
			if(a==1 && b==0 && (c == 0 || c==1))
			{
			tmp = 33;
			}
			else
			{
				tmp = 0;
			}
		}
	}
	return tmp;	
}


float finalApproaching::objectifY()
{
	float tmp=0;
	switch(m_parameter)
	{
		case manager_msg::finalApproachingGoal::S1 :
			tmp = 0.28; break;
		case manager_msg::finalApproachingGoal::S2 :
			tmp = 0.175; break;
		case manager_msg::finalApproachingGoal::S3 :
			tmp = 0.08; break;
		case manager_msg::finalApproachingGoal::LANE_RS :
				tmp = 0.09; break;
		case manager_msg::finalApproachingGoal::LIGHT :
			tmp = 0.4; break;
		case manager_msg::finalApproachingGoal::CONVEYOR :
			switch(m_side)
			{
				case manager_msg::finalApproachingGoal::IN :
					tmp = 0.37; break;
				case manager_msg::finalApproachingGoal::OUT :
					tmp = 0.32; break;
			}
		default: break;
	}
	return tmp;
}

float finalApproaching::objectifX()
{
	float tmp = 0;
	if(m_parameter == manager_msg::finalApproachingGoal::LIGHT)
	{
		tmp = 0.35;
	}
	else
	{
		tmp = 0.14;
	}
	return tmp;
}
