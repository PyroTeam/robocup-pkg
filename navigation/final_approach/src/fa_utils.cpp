#include "final_approach/fa_utils.h"

#include "final_approach/Point.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <final_approach_msg/plotDataFA.h>

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


float distance2points(Point a, Point b)
{
	return sqrt(  std::abs(b.getX() - a.getX()) * std::abs(b.getX() - a.getX())
				+ std::abs(b.getY() - a.getY()) * std::abs(b.getY() - a.getY()));
}

float mean(std::list<float> position_y){
	std::list<float>::iterator it;
	float moyenne = 0;
	for(it = position_y.begin(); it != position_y.end(); it++)
	{
		moyenne = moyenne + *it;
	}
	moyenne = moyenne / (float)(position_y.size());
	return moyenne;
}
