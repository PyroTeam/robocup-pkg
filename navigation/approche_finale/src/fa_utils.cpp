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
	std::cout<<"test dist2pts 1"<<std::endl;
	float xa = a.getx();
	std::cout<<"test dist2pts 2"<<std::endl;
	float xb = b.getx();
	std::cout<<"test dist2pts 3"<<std::endl;
	float ya = a.gety();
	std::cout<<"test dist2pts 4"<<std::endl;
	float yb = b.gety();
	std::cout<<"test dist2pts 5"<<std::endl;
	return sqrt( std::abs(xb-xa)*std::abs(xb-xa) + std::abs(yb-ya)*std::abs(yb-ya) );
}

float moy(std::list<float> position_y){
	std::list<float>::iterator it;
	float moyenne = 0;
	for(it=position_y.begin();it!=position_y.end();it++){
		moyenne = moyenne + *it;
	}
	moyenne = moyenne / (float)(position_y.size()) ;
	return moyenne;
}

int asservissement_angle(ros::Publisher pub_mvt,float angle){
	geometry_msgs::Twist msg;
	
	angle = fmod(angle,2*M_PI);
	if(std::abs(angle) < 0.03){
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
		if(std::abs(moy_pos - objectif) < 0.01){
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

int asservissement_position_x(ros::Publisher pub_mvt, float distance, float objectif){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	if(std::abs(distance-objectif) < 0.01) {
		msg.linear.x = 0;
		pub_mvt.publish(msg);
		return 2;
	}
	else{
		if(std::abs(distance-objectif) < 0.04){
			msg.linear.x = (distance - objectif)*0.25;
			pub_mvt.publish(msg);
			return 1;
		}
		else{
			msg.linear.x = (distance - objectif)*0.25;
			pub_mvt.publish(msg);
			return 0;
		}
	}
}


