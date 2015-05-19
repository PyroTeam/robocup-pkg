#include <cmath>
#include <vector>
#include <functional>
#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include "fa_utils.h"
#include "Point.h"
#include "geometry_msgs/Twist.h"


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

int asservissementAngle(ros::Publisher pubMvt,float angle){
	geometry_msgs::Twist msg;
	angle = fmod(angle,2*M_PI);
	if(std::abs(angle) < 0.03)
	{
		msg.angular.z = 0;
		pubMvt.publish(msg);
		return 1;
	}
	else
	{
		msg.angular.z = -0.75*angle;
		pubMvt.publish(msg);
		return 0;
	}
	
}

int asservissementPositionY(ros::Publisher pubMvt, float moyPos,float goal, float ortho){
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	/*if(ortho < 0)
	{
		if(ortho == -1)
		{
			msg.linear.y = 0.25;
			pubMvt.publish(msg);
			return 0;
		}
		if(ortho == -2)
		{
			msg.linear.y = -0.25;
			pubMvt.publish(msg);
			return 0;
		}
	}
	else*/ 
	{
		if(std::abs(moyPos - goal) < 0.01)
		{
			msg.linear.y = 0;
			pubMvt.publish(msg);
			return 1;
		}
		else
		{
			msg.linear.y = -0.5*(moyPos - goal);
			pubMvt.publish(msg);
			return 0;
		}
	}
}

int asservissementPositionX(ros::Publisher pubMvt, float distance, float goal){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	if(std::abs(distance-goal) < 0.01) 
	{
		msg.linear.x = 0;
		pubMvt.publish(msg);
		return 2;
	}
	else
	{
		if(std::abs(distance-goal) < 0.04)
		{
			msg.linear.x = (distance - goal)*0.25;
			pubMvt.publish(msg);
			return 1;
		}
		else
		{
			msg.linear.x = (distance - goal)*0.25;
			pubMvt.publish(msg);
			return 0;
		}
	}
}


