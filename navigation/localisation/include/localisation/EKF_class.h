#ifndef EKF_CLASS_H
#define EKF_CLASS_H

#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

#include "deplacement_msg/Landmarks.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

using namespace Eigen;

class EKF{
public:
	EKF();

	void set();
	void printZones();

	//callbacks
	void odomCallback(const nav_msgs::Odometry& odom);
	void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines);
	void machinesVuesCallback(const deplacement_msg::LandmarksConstPtr& machines);
	void laserCallback(const deplacement_msg::LandmarksConstPtr& laser);

	VectorXd RobotToLaser(VectorXd PosRobot);
	VectorXd GlobalToRobot(VectorXd p);

	geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);
	geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p);
	geometry_msgs::Pose2D getCenter(int zone);

	void addMachine(geometry_msgs::Pose2D machine);

	int checkStateVector(geometry_msgs::Pose2D machine);
	int getZone(geometry_msgs::Pose2D m);
	int machineToArea(geometry_msgs::Pose2D m);
	double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m);

	void updateP(const MatrixXd &Pm, int i);
	void updatePprev(const MatrixXd &Pm, int i);
	void updateXmean(const VectorXd &tmp, int i);

	MatrixXd buildPm(int i);
	MatrixXd buildH(int i);
	MatrixXd buildH2(geometry_msgs::Pose2D p, int taille, int i);

	void prediction();
	void correction(geometry_msgs::Pose2D p, int posMachineInTabMachines);

	VectorXd getXmean();
	VectorXd getXpredicted();

	std::vector<int> getZones();
	std::vector<geometry_msgs::Pose2D> getScan();
	std::vector<geometry_msgs::Pose2D> getTabMachines();
	
	void setZone(int i){m_zones.push_back(i);}
	void fillMachines();
private:
	geometry_msgs::Pose2D m_odomRobot;

	VectorXd m_xMean;
	MatrixXd m_P;
	MatrixXd m_P_prev;

	VectorXd m_xPredicted;
	VectorXd m_cmdVel;

	std::vector<geometry_msgs::Pose2D> 	m_tabMachines;
	std::vector<geometry_msgs::Pose2D> 	m_tabLandmarks;
	std::vector<geometry_msgs::Pose2D> 	m_scan;
	std::vector<int>					m_zones;

	ros::Time m_temps;
};

#endif