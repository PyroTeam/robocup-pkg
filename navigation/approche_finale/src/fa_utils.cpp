#include <cmath>
#include <vector>
#include <functional>
#include <ros/ros.h>
#include <math.h>

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

int asservissement_angle(ros::Publisher pub_mvt,float angle){
	geometry_msgs::Twist msg;
	
	angle = fmod(angle,2*M_PI);
	if(std::abs(angle) < 0.05){
		msg.angular.z = 0;
		pub_mvt.publish(msg);
		return 1;
		}
	else{
		
			msg.angular.z = -0.75*angle;
		
	pub_mvt.publish(msg);
	return 0;
	}
	
}

int asservissement_position_y(ros::Publisher pub_mvt, float moy_pos,float objectif, float ortho){
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	/*if(ortho < 0){
		if(ortho == -1){
				msg.linear.y = 0.25;
				pub_mvt.publish(msg);
				return 0;
			}
			if(ortho == -2){
				msg.linear.y = -0.25;
				pub_mvt.publish(msg);
				return 0;
			}
	}
	else*/ {
		if(std::abs(moy_pos - objectif) < 0.02){
			msg.linear.y = 0;
			pub_mvt.publish(msg);
			return 1;
			}
		else{
			msg.linear.y = -0.5*(moy_pos - objectif);
			pub_mvt.publish(msg);
			return 0;
		}
	}
}

int asservissement_position_x(ros::Publisher pub_mvt, float distance){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	if(distance < 0.16) {
		msg.linear.x = 0;
		pub_mvt.publish(msg);
		return 1;
	}
	else{
		msg.linear.x = std::abs(distance - 0.16)*0.25;
		pub_mvt.publish(msg);
		return 0;
	}
}
	
	
	
			
