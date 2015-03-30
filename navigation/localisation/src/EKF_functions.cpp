#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

#include "EKF_functions.h"

void initPosRobot(string s,int n){
	int color = 0;
	if (s == "cyan"){
		color = 1;
	}
	else {
		color = -1;
	}

	initRobot.theta =  0.0;
	initRobot.y 	= -0.5;

	switch(n){
		case 1 :
			initRobot.x = color*3.5;
		break;
		case 2 :
			initRobot.x = color*4.5;
		break;
		case 3 :
			initRobot.x = color*5.5;
		break;
		default :
		break;
	}
}

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser){
	geometry_msgs::Pose2D p;
	Matrix3d m;
	m.setZero();
	Vector2d before;
	Vector2d after;

	before(0) = PosRobot.x;
	before(1) = PosRobot.y;

	std::cout << "the position is:" << std::endl << m << std::endl;

	//le laser se situe 30 cm en avant par rapport au centre de gravité du robot
	m(1,1) = 0.3;

	after = before*m;

	std::cout << "Now the position is:" << std::endl << after << std::endl;

	p.x = after(0) ;
	p.y = after(1) ;
	p.theta = PosLaser.theta;

	return p;
}

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D PosInitRobot, geometry_msgs::Pose2D odomRobot){
	geometry_msgs::Pose2D p;
	Matrix3d m;
	m.setZero();
	Vector3d before;
	Vector3d after;

	before(0) = odomRobot.x + PosInitRobot.x;
	before(1) = odomRobot.y + PosInitRobot.y;
	before(2) = 0;

	//translation
	m(0,2) = PosInitRobot.x;
	m(1,2) = PosInitRobot.y;
	//rotation
	m(0,0) =  cos(odomRobot.theta*M_PI/180);
	m(0,1) = -sin(odomRobot.theta*M_PI/180);
	m(1,0) =  sin(odomRobot.theta*M_PI/180);
	m(1,1) =  cos(odomRobot.theta*M_PI/180);

	m(2,2) = 1;

	after = m * before;

	p.x 	= after(0);
	p.y 	= after(1);
	p.theta = odomRobot.theta;

	return p;
}

void addMachine(geometry_msgs::Pose2D machine){

}