#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <eigen/dense>

#include "EKF_functions.h"

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

	before(0) = PosRobot.x;
	before(1) = PosRobot.y;
	before(2) = 0;

	double angle = atan2(PosRobot.y, PosRobot.x) + PosRobot.theta;

	//translation
	p(0,2) = init.;
	p(1,2) = ;
	//rotation
	p(0,0) = ;
	p(0,1) = ;
	p(1,0) = ;
	p(1,1) = ;

	p(2,2) = 1;

	p.x 	= after(0);
	p.y 	= after(1);
	p.theta = after(2);

	return p;
}