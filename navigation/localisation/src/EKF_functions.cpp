#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>
#include <limits>

#include "EKF_functions.h"

using namespace Eigen;

void correctAngle(double &angle){
	double dx = 0;
	while (angle > M_PI || angle <= -M_PI){
		if (angle > M_PI){
			dx = M_PI - angle;
			angle = -M_PI + dx;
		}
		else if (angle <= -M_PI){
			dx = -M_PI - angle;
			angle = M_PI - dx;
		}
	}
}

void correctStateVector(VectorXd &xMean){
	for (int i = 2; i < xMean.rows(); i=i+3){
		correctAngle(xMean(i));
	}
}

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

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, 
									geometry_msgs::Pose2D PosInitRobot,
									geometry_msgs::Pose2D odomRobot){
	Vector3d before, after;
	Matrix3d m;
	double angle = odomRobot.theta;
	correctAngle(angle);

	before(0) = p.x;
	before(1) = p.y;
	before(2) = 1; //toujours 1 ici !

	m.setZero();
	//translation
	m(0,2) = PosInitRobot.x + odomRobot.x;
	m(1,2) = PosInitRobot.y + odomRobot.y;
	//rotation
	Matrix2d rot;
 	rot = Rotation2Dd(angle);
 	m.topLeftCorner(2,2) = rot;

	m(2,2) = 1;

	after = m*before;

	p.x 	= after(0);
	p.y 	= after(1);
	p.theta = angle;

	return p;
}

Vector3d cmdVelDansGlobal(Vector3d cmdVel, double angle){
	Vector3d tmp;
	Matrix3d m;
	m.setZero();

	Matrix2d rot;
 	rot = Rotation2Dd(angle);
 	m.topLeftCorner(2,2) = rot;
	m(2,2) = 1;

	tmp = m*cmdVel;

	return tmp;
}

void addMachine(geometry_msgs::Pose2D machine, VectorXd &xMean, MatrixXd &P){
	//on redimensionne xMean et P pour accueillir la nouvelle machines
	xMean.conservativeResize(xMean.rows() + 3);
	P.conservativeResize(P.rows()+3,P.cols()+3);

	//on remplit avec les coordonnées de la nouvelle machine
	xMean(xMean.rows()-3) = machine.x;
	xMean(xMean.rows()-2) = machine.y;
	xMean(xMean.rows()-1) = machine.theta;
	correctAngle(xMean(xMean.rows()-1));

	//calcul de tous les PLi
	//initialisation des PLi à 0
	P.block(P.rows() - 3, 0, 3, P.cols()).setZero();
	P.block(0, P.cols() - 3, P.rows(), 3).setZero();

	for (int j = 0; j < xMean.rows(); j = j + 3){
		//position de la nouvelle machines par rapport au robot
		//et à toutes les autres
		double x 	 = xMean(xMean.rows()-3) - xMean(j  );
		double y 	 = xMean(xMean.rows()-2) - xMean(j+1);
		double theta = xMean(xMean.rows()-1) - xMean(j+2);

		P(P.rows()-3, j  ) = x;
		P(P.rows()-2, j+1) = y;
		P(P.rows()-1, j+2) = theta;

		P(j  , P.rows()-3) = x;
		P(j+1, P.rows()-2) = y;
		P(j+2, P.rows()-1) = theta;

		P(j  , j  ) = xMean(j  ) - xMean(0);
		P(j+1, j+1) = xMean(j+1) - xMean(1);
		P(j+2, j+2) = xMean(j+2) - xMean(2);
	}
}

int checkStateVector(const VectorXd &xMean, geometry_msgs::Pose2D machine){
    std::cout << "taille de xMean :" << xMean.rows() << "\n" << std::endl;
 	for (int i = 3; i < xMean.rows(); i=i+3){
 		double d = sqrt((machine.x-xMean(i))*(machine.x-xMean(i)) +
 				 		(machine.y-xMean(i+1))*(machine.y-xMean(i+1)));
 		std::cout << "la distance entre machines est de " << d << std::endl;
 		if (d < 2.0) return i;
 	}

 	return 0;
}

MatrixXd buildPm(MatrixXd P, int i){
	MatrixXd Pm(6,6);
	Pm.setZero();

	Pm.block(0,0,3,3) = P.block(0,0,3,3);
 	Pm.block(3,3,3,3) = P.block(i,i,3,3);
 	Pm.block(0,3,3,3) = P.block(0,i,3,3);
 	Pm.block(3,0,3,3) = P.block(i,0,3,3); 	

 	return Pm;
}

void updateP(MatrixXd &P, MatrixXd Pm, int i){
	P.block(0,0,3,3) = Pm.block(0,0,3,3);
 	P.block(i,i,3,3) = Pm.block(3,3,3,3);
 	P.block(0,i,3,3) = Pm.block(0,3,3,3);
 	P.block(i,0,3,3) = Pm.block(3,0,3,3); 	
}

void updateXmean(VectorXd &xMean, VectorXd tmp, int i){
	xMean.block(0,0,3,1) = tmp.block(0,0,3,1);
	xMean.block(i,0,3,1) = tmp.block(3,0,3,1);
}

MatrixXd buildH(VectorXd xMean, int taille, int i){
	MatrixXd H(3,6);
	H.setZero();
	H.block(0,0,3,3) = MatrixXd::Identity(3,3);
	H(0,3) = xMean(0) - xMean(i  );
	H(1,4) = xMean(1) - xMean(i+1);
	H(2,5) = xMean(2) - xMean(i+2);

	return H;
}

MatrixXd buildH2(VectorXd xMean, int taille, int i){
	MatrixXd H(3,taille);
	H.setZero();
	H.block(0,0,3,3) = MatrixXd::Identity(3,3);
	H(0,i  ) = xMean(0) - xMean(i  );
	H(1,i+1) = xMean(1) - xMean(i+1);
	H(2,i+2) = xMean(2) - xMean(i+2);

	return H;
}

Vector3d prediction(VectorXd &xMean, MatrixXd &P, Vector3d cmdVel, ros::Time &temps){
	std::cout << "prediction\n" << std::endl;

	//calcul de la période pour la prédiction
	ros::Duration duree = ros::Time::now() - temps;
	double periode = duree.toSec();

	double angle = xMean(2);
	correctAngle(angle);
	Vector3d cmd = cmdVelDansGlobal(cmdVel, angle);

	//calcul de la position du robot pour l'instant n+1
	Vector3d xPredicted = xMean.topLeftCorner(3,1) + periode*cmd;
	std::cout << "xPredicted = \n" << xPredicted << std::endl;
	std::cout << "periode = " << periode << std::endl;

	MatrixXd Fx = MatrixXd::Identity(P.rows(),P.cols());
	Fx(0,0) = xPredicted(0) - xMean(0);
	Fx(1,1) = xPredicted(1) - xMean(1);
	Fx(2,2) = xPredicted(2)	- xMean(2);
	//std::cout << "Fx = \n" << Fx << std::endl;

	xMean.topLeftCorner(3,1) = xPredicted;
	correctAngle(xMean(2));

	//mise à jour de P
	P = Fx*P*(Fx.transpose());
	//mise à jour du temps
	temps = ros::Time::now();

	return xPredicted;
}

void correction(VectorXd &xMean, MatrixXd &P, Vector3d xPredicted, int posMachineInStateVector){
	std::cout << "correction\n" << std::endl;

	int taille = P.rows();

	//calcul de z
	Vector3d z;
	z.setZero();
	z = xPredicted - xMean.block(xMean.rows()-3,0,3,1);
	std::cout << "z = \n" << z << "\n" << std::endl;

	//calcul de H
	MatrixXd H(3, taille);
	H = buildH(xMean, taille, posMachineInStateVector);
	std::cout << "H = \n" << H << "\n" <<  std::endl;

	//calcul de R
	MatrixXd R(3,3);
	R.setZero();
	R(0,0) = xMean(0);
	R(1,1) = xMean(1);
	R(2,2) = xMean(2);

	//calcul de Pm
	MatrixXd Pm = buildPm(P, posMachineInStateVector);
	std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;

	//calcul de Z
	MatrixXd Z(taille, taille);
	Z.setZero();
	Z = H*Pm*(H.transpose()) + R;
	std::cout << "Z = \n" << Z << "\n" <<  std::endl;

	//calcul du gain de Kalman
	MatrixXd K(3, 3);
	K.setZero();
	K = Pm*(H.transpose())*(Z.inverse());
	std::cout << "K = \n" << K << "\n" <<  std::endl;

	//mise à jour du vecteur xMean
	VectorXd tmp(6,1);
	tmp.block(0,0,3,1) = xMean.block(0,0,3,1);
	tmp.block(3,0,3,1) = xMean.block(posMachineInStateVector,0,3,1);
	//std::cout << "tmp = \n" << tmp << "\n" <<  std::endl;
	tmp +=  K*z;
	updateXmean(xMean, tmp, posMachineInStateVector);
	correctStateVector(xMean);
	//std::cout << "position du robot corrigee = \n" <<  xMean.topLeftCorner(3,1) << "\n" << std::endl;

	//mise à jour de la matrice P
	Pm = Pm - K*Z*(K.transpose());
	//std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;
	//std::cout << "KZKt = \n" << K*Z*(K.transpose()) << "\n" <<  std::endl;
	updateP(P, Pm, posMachineInStateVector);

}