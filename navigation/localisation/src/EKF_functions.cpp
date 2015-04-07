#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

#include "EKF_functions.h"

using namespace Eigen;

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser){
	geometry_msgs::Pose2D p;
	Matrix3d m;
	m.setZero();
	Vector3d before;
	Vector3d after;

	//translation
	m(1,2) = 0.2;
	//rotation
	Matrix2d rot;
 	rot = Rotation2Dd(M_PI_2);
 	m.topLeftCorner(2,2) = rot;
	m(2,2) = 1;

	before(0) = PosLaser.x;
	before(1) = PosLaser.y;
	before(2) = 1;

 	std::cout << m << std::endl;

	after = m*before;

	p.x = after(0) ;
	p.y = after(1) ;
	//on veut l'angle machine dans [0, 2*PI]
	if (PosLaser.theta >= 0){
		p.theta = PosLaser.theta - M_PI_2;
	}
	else {
		p.theta = PosLaser.theta + M_PI_2;
	}

	return p;
}

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D PosInitRobot, geometry_msgs::Pose2D odomRobot){
	geometry_msgs::Pose2D p;
	Vector3d before, after;
	Matrix3d m;

	before(0) = odomRobot.x + PosInitRobot.x;
	before(1) = odomRobot.y + PosInitRobot.y;
	before(2) = 1; //toujours 1 ici !

	m.setZero();
	//translation
	m(0,2) = PosInitRobot.x;
	m(1,2) = PosInitRobot.y;
	//rotation
	Matrix2d rot;
 	rot = Rotation2Dd(odomRobot.theta);
 	m.topLeftCorner(2,2) = rot;

	m(2,2) = 1;

	after = m*before;

	p.x 	= after(0);
	p.y 	= after(1);
	p.theta = odomRobot.theta;

	return p;
}

Vector3d Pose2DToVector(geometry_msgs::Pose2D p){
	Vector3d tmp;
	tmp(0) = p.x;
	tmp(1) = p.y;
	tmp(2) = p.theta;

	return tmp;
}

void mapInitialization(VectorXd &xMean, MatrixXd &P){
	xMean(3);
	xMean.setZero();
	P(3,3);
	P.setZero();
}

void addMachine(geometry_msgs::Pose2D machine, VectorXd &xMean, MatrixXd &P){
	//on redimensionne xMean et P pour accueillir la nouvelle machines
	xMean.conservativeResize(xMean.rows() + 3);
	P.conservativeResize(P.rows() + 3,P.rows() + 3);
	//on remplit avec les coordonnées de la nouvelle machine
	xMean(xMean.rows()-3) = machine.x;
	xMean(xMean.rows()-2) = machine.y;
	xMean(xMean.rows()-1) = machine.theta;

	//plus compliqué que ça
	P.bottomRightCorner(3,3);
}

int checkStateVector(VectorXd xMean, geometry_msgs::Pose2D machine){
 	for (int i = 3; i < xMean.rows(); i=i+3){
 		if (std::abs(machine.x - xMean(i)) < 0.5 &&	std::abs(machine.y - xMean(i+1)) < 0.5){
 			return i;
 		}
 	}

 	return 0;
}

MatrixXd buildPm(MatrixXd P, int i){
	MatrixXd Pm(6,6);
	Pm.setZero();

	Pm.topLeftCorner(3,3) 		= P.topLeftCorner(3,3);
 	Pm.bottomRightCorner(3,3) 	= P.block(i,i,3,3);
 	Pm.topRightCorner(3,3)		= P.block(0,i,3,3);
 	Pm.bottomLeftCorner(3,3) 	= P.block(i,0,3,3); 	

 	return Pm;
}

Vector3d predict(VectorXd xMean, MatrixXd &P, geometry_msgs::Pose2D cmdVel){
	Vector3d xPredicted, cmdVelVect;
	cmdVelVect = Pose2DToVector(cmdVel);
 	xPredicted.setZero();

	int periode = 2;

	xPredicted = xMean.topLeftCorner(3,1) + periode*cmdVelVect;

	return xPredicted;
}