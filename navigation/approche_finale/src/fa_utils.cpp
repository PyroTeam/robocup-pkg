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

int asservissement_angle(ros::Publisher pub_mvt,float moy_pente){
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	if(std::abs(moy_pente) < 0.1){
		msg.angular.z = 0;
		pub_mvt.publish(msg);
		return 1;
		}
	else{
		if(moy_pente < - 0.1){
			//tourner sens trigo
			msg.angular.z = 1;
		}
		if(moy_pente > 0.1){
			//tourner sens anti trigo
			msg.angular.z = -1;
		}
	pub_mvt.publish(msg);
	return 0;
	}
	
}

int asservissement_position_y(ros::Publisher pub_mvt, float moy_pos,float objectif){
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.angular.z = 0;
	if(std::abs(moy_pos - objectif) > 0.03){
		msg.linear.y = 0;
		pub_mvt.publish(msg);
		return 1;
		}
	else{
		if(moy_pos - objectif < - 0.03){
			//aller vers la droite
			msg.linear.y = -1;
		}
		if(moy_pos - objectif > 0.03){
			//aller vers la gauche
			msg.linear.y = 1;
		}
	pub_mvt.publish(msg);
	return 0;
	}
}

int asservissement_position_x(ros::Publisher pub_mvt, float distance){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	if(distance < 0.15) {
		msg.linear.x = 0;
		pub_mvt.publish(msg);
		return 1;
	}
	else{
		msg.linear.x = 1;
		pub_mvt.publish(msg);
		return 0;
	}
}
	
	
	
			
