#ifndef EKF_FUNCTIONS_H
#define EKF_FUNCTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

void correctAngle(double &angle);

void correctStateVector(VectorXd &xMean);

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, 
									geometry_msgs::Pose2D PosInitRobot,
									geometry_msgs::Pose2D odomRobot);

Vector3d cmdVelDansGlobal(Vector3d cmdVel, double angle);

void addMachine(geometry_msgs::Pose2D machine,
				VectorXd &xMean,
				MatrixXd &P);

int checkStateVector(const VectorXd &xMean, geometry_msgs::Pose2D machine);

MatrixXd buildPm(MatrixXd P, int i);

void updateP(MatrixXd &P, MatrixXd Pm, int i);

void updateXmean(VectorXd &xMean, VectorXd tmp, int i);

MatrixXd buildH(VectorXd xMean, int taille, int i);

MatrixXd buildH2(VectorXd xMean, int taille, int i);

Vector3d prediction(VectorXd &xMean,
					MatrixXd &P,
					Vector3d cmdVel,
					ros::Time &temps);

void correction(VectorXd &xMean,
				MatrixXd &P,
				Vector3d xPredicted,
				int posMachineInTabMachines);

#endif