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

int checkStateVector(VectorXd xMean, geometry_msgs::Pose2D machine){
 	for (int i = 3; i < xMean.rows(); i=i+3){
 		if (std::abs(machine.x - xMean(i)) < 0.5 &&	std::abs(machine.y - xMean(i+1)) < 0.5){
 			return i;
 		}
 	}

 	return 0;
}

MatrixXd buildPm(MatrixXd P, int i){
	MatrixXd Pm(P.rows(),P.cols());
	Pm.setZero();

	Pm.block(0,0,3,3) = P.block(0,0,3,3);
 	Pm.block(i,i,3,3) = P.block(i,i,3,3);
 	Pm.block(0,i,3,3) = P.block(0,i,3,3);
 	Pm.block(i,0,3,3) = P.block(i,0,3,3); 	

 	return Pm;
}

MatrixXd buildH(int taille, int i){
	MatrixXd H(3,taille);
	H.block(0,0,3,3) = MatrixXd::Identity(3,3);
	H.block(0,i,3,3) = MatrixXd::Identity(3,3);

	return H;
}

Vector3d prediction(VectorXd xMean, MatrixXd &P, geometry_msgs::Pose2D cmdVel, ros::Time &temps){
	Vector3d cmdVelVect = Pose2DToVector(cmdVel);

	//calcul de la période pour la prédiction
	ros::Duration duree = ros::Time::now() - temps;
	double periode = duree.toSec();

	//calcul de la position du robot pour l'instant n+1
	Vector3d xPredicted = xMean.topLeftCorner(3,1) + periode*cmdVelVect;

	MatrixXd Fx;
	Fx = MatrixXd::Identity(P.rows(), P.cols());
	Fx(0,0) = xPredicted(0);
	Fx(1,1) = xPredicted(1);
	Fx(2,2) = xPredicted(2);

	//mise à jour de P
	P = Fx*P*(Fx.transpose());
	//mise à jour du temps
	temps = ros::Time::now();

	return xPredicted;
}

void correction(VectorXd &xMean, MatrixXd &P, Vector3d xPredicted, geometry_msgs::Pose2D m){
	int taille = P.rows();
	int i = checkStateVector(xMean, m);

	//calcul de H
	MatrixXd H(3, taille);
	H = buildH(taille, i);
	//calcul de Pm
	MatrixXd Pm(taille, taille);
	Pm = buildPm(P, i);
	//calcul de Z
	MatrixXd Z(taille, taille);
	Z = H*Pm*(H.transpose());

	//calcul du gain de Kalman
	MatrixXd K(taille, taille);
	K = P*(H.transpose())*(Z.inverse());

	//mise à jour du vecteur xMean
	xMean += K*(xMean.block(xMean.rows()-3,0,3,1) - xPredicted);
	//mise à jour de la matrice P
	P -= K*Z*(K.transpose());
}

void addMachine(geometry_msgs::Pose2D machine, VectorXd &xMean, MatrixXd &P){
	//on redimensionne xMean et P pour accueillir la nouvelle machines
	xMean.conservativeResize(xMean.rows() + 3);
	P.conservativeResize(P.rows() + 3,P.cols() + 3);
	
	//on remplit avec les coordonnées de la nouvelle machine
	xMean(xMean.rows()-3) = machine.x;
	xMean(xMean.rows()-2) = machine.y;
	xMean(xMean.rows()-1) = machine.theta;

	//calcul de tous les PLi
	//initialisation des PLi à 0
	P.block(P.rows() - 3, 0, 3, P.cols()).setZero();
	P.block(0, P.cols() - 3, P.rows(), 3).setZero();

	double x, y, theta;

	//remplissage avec les vecteurs de positionnement
	for (int j = 0; j < xMean.rows(); j = j + 3){
		//dérivées partielles suivant x, y et theta
		x 	  = xMean(xMean.rows()-3) - xMean(j  );
		y 	  = xMean(xMean.rows()-2) - xMean(j+1);
		theta = xMean(xMean.rows()-1) - xMean(j+2);

		P(P.rows()-3, j  ) = x;
		P(P.rows()-2, j+1) = y;
		P(P.rows()-1, j+2) = theta;

		P(j  , P.rows()-3) = x;
		P(j+1, P.rows()-2) = y;
		P(j+2, P.rows()-1) = theta;
	}
}