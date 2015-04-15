#include <cmath>
#include <vector>
#include <ros/ros.h>

#include "fa_utils.h"
#include "Point.h"
#include "geometry_msgs/Twist.h"


float distance2points(Point a, Point b){
	float xa = a.getx();
	float xb = b.getx();
	float ya = a.gety();
	float yb = b.gety();
	return sqrt( std::abs(xb-xa)*std::abs(xb-xa) + std::abs(yb-ya)*std::abs(yb-ya) );
}

float moy(std::vector<float> position_y){
	std::vector<float>::iterator it;
	float moyenne = 0;
	for(int i=0;i<position_y.size();i++){
		moyenne = moyenne + position_y[i];
	}
	moyenne = moyenne / (float)(position_y.size()) ;
	return moyenne;
}

void asservissement_angle(ros::NodeHandle n,float moy_pente){
	ros::Publisher angle = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	geometry_msgs::Twist msg;
	if(moy_pente > -0.1 && moy_pente < 0.1)
			msg.angular.z = 0;
	else{
		if(moy_pente < - 0.1){
			//tourner sens trigo
			msg.angular.z = 1;
		}
		if(moy_pente > 0.1){
			//tourner sens anti trigo
			msg.angular.z = -1;
		}
	}
	angle.publish(msg);
}		
