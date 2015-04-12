#ifndef EKF_FUNCTIONS_H
#define EKF_FUNCTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p,
									geometry_msgs::Pose2D PosInitRobot,
									geometry_msgs::Pose2D odomRobot);

Vector3d Pose2DToVector(geometry_msgs::Pose2D p);

void addMachine(geometry_msgs::Pose2D machine,
				VectorXd &xMean,
				MatrixXd &P);

int checkStateVector(VectorXd xMean,
					 geometry_msgs::Pose2D machine);

MatrixXd buildPm(MatrixXd P, int i);

MatrixXd buildH(int taille, int i);

Vector3d prediction(VectorXd xMean,
					MatrixXd &P,
					geometry_msgs::Pose2D cmdVel,
					ros::Time &temps);

void correction(VectorXd &xMean,
				MatrixXd &P,
				Vector3d xPredicted,
				geometry_msgs::Pose2D m);

int chooseMachine(std::vector<geometry_msgs::Pose2D> tabMachines,
				  VectorXd xMean);

#endif