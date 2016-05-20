#include "fa_utils.h"

#include "Point.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <approche_finale_msg/plotDataFA.h>

#include <cmath>
#include <vector>
#include <functional>
#include <math.h>
#include <iostream>

// XXX: Make a common lib with PID or look after http://wiki.ros.org/pid and http://wiki.ros.org/lyap_control

geometry_msgs::Point orthoProjection(Point p, geometry_msgs::Pose2D p2d)
{
        float denom = sqrt(cos(p2d.theta)*cos(p2d.theta)+sin(p2d.theta)*sin(p2d.theta));
        float num = (p.getX()-p2d.x)*cos(p2d.theta)+(p.getY()-p2d.y)*sin(p2d.theta);
        float H = num/denom;
        geometry_msgs::Point ortho;
        ortho.x = (float)(p2d.x + (H/denom)*cos(p2d.theta));
        ortho.y = (float)(p2d.y + (H/denom)*sin(p2d.theta));
        ortho.z = 0;
        return ortho;
}


float distance2points(Point a, Point b){
	float xa = a.getX();
	float xb = b.getX();
	float ya = a.getY();
	float yb = b.getY();
	return sqrt( std::abs(xb-xa)*std::abs(xb-xa) + std::abs(yb-ya)*std::abs(yb-ya) );
}

float moy(std::list<float> position_y){
	std::list<float>::iterator it;
	float moyenne = 0;
	for(it=position_y.begin();it!=position_y.end();it++)
	{
		moyenne = moyenne + *it;
	}
	moyenne = moyenne / (float)(position_y.size()) ;
	return moyenne;
}

int asservissementAngle(approche_finale_msg::plotDataFA &plotData,ros::Publisher pubMvt,float angle){
	geometry_msgs::Twist msg;
	plotData.angleErr = std::abs(angle-0.01);
	if(std::abs(angle-0.01) < 0.015)
	{
		msg.angular.z = 0;
		plotData.angleCmd = 0;
		pubMvt.publish(msg);
		return 1;
	}
	else
	{
		msg.angular.z = 0.4*(angle-0.01);
		plotData.angleCmd = 0.4*(angle-0.01);
		pubMvt.publish(msg);
		return 0;
	}
	
}


int asservissementPositionY(approche_finale_msg::plotDataFA &plotData,ros::Publisher pubMvt, float goal, float moyPos, float yLeft, float yRight){
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	if(yLeft >= 0 && yRight >= 0)
	{
		ROS_DEBUG("Il faut tourner aller a droite");
		msg.linear.y = 0.25;
		plotData.YErr = 2; // 2 >> error
        plotData.YCmd = 0.25;
        pubMvt.publish(msg);		
		return 0;
	}
	else
	{
		if(yLeft <= 0 && yRight <= 0)
		{
			ROS_DEBUG("Il faut aller a gauche");
			msg.linear.y = -0.25;
			plotData.YErr = 2; // 2 >> error
			plotData.YCmd = -0.25;
	        pubMvt.publish(msg);
			return 0;
		}
		else
		{
			ROS_INFO("deplacement normal");
			plotData.YErr = std::abs(moyPos - goal);
			if(std::abs(moyPos + goal) < 0.003)
			{
				msg.linear.y = 0;
				plotData.YCmd = 0;
				pubMvt.publish(msg);
				return 1;
			}
			else
			{
				msg.linear.y = 0.025*(moyPos + goal);
				plotData.YCmd = 0.025*(moyPos + goal);
				pubMvt.publish(msg);
				return 0;
			}
		}
	}
}


int asservissementPositionX(approche_finale_msg::plotDataFA &plotData,ros::Publisher pubMvt, float distance, float goal){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	plotData.XErr = std::abs(distance-goal);
	if(std::abs(distance-goal) < 0.007) 
	{
		msg.linear.x = 0;
		plotData.XCmd = 0;
		pubMvt.publish(msg);
		return 2;
	}
	else
	{
		if(std::abs(distance-goal) < 0.04)
		{
			msg.linear.x = (distance - goal)*0.25;
			plotData.XCmd = (distance - goal)*0.25;
			pubMvt.publish(msg);
			return 1;
		}
		else
		{
			msg.linear.x = (distance - goal)*0.25;
			plotData.XCmd = (distance - goal)*0.25;
			pubMvt.publish(msg);
			return 0;
		}
	}
}


