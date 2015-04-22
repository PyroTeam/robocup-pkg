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
#include "EKF_functions.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

using namespace Eigen;

class EKF{
public:
	EKF(std::string s, int num);

	int machineToArea(geometry_msgs::Pose2D machine);

	//callbacks
	void odomCallback(const nav_msgs::Odometry& odom);
	void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines);
	void laserCallback(const deplacement_msg::LandmarksConstPtr& laser);

	void correctAngle(double &angle);
	void correctStateVector();

	geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);
	geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p);
	
	void cmdVelDansGlobal(double angle);

	void addMachine(geometry_msgs::Pose2D machine);

	int checkStateVector(geometry_msgs::Pose2D machine);

	MatrixXd buildPm(int i);

	void updateP(const MatrixXd &Pm, int i);
	void updateXmean(const VectorXd &tmp, int i);

	MatrixXd buildH(int i);
	MatrixXd buildH2(int taille, int i);

	void prediction();
	void correction(geometry_msgs::Pose2D p, int posMachineInTabMachines);

	VectorXd getXmean() {return m_xMean;}
	std::vector<int> getZones() {return m_zones;}
	std::vector<geometry_msgs::Pose2D> getScan(){return m_scan;}

	std::vector<geometry_msgs::Pose2D> getTabMachines() {return m_tabMachines;}
	void setZone(int i){m_zones.push_back(i);}
	bool test(int area);
	void printZones();
private:
	geometry_msgs::Pose2D m_odomRobot;
	geometry_msgs::Pose2D m_initRobot;

	VectorXd m_xMean;
	MatrixXd m_P;

	Vector3d m_xPredicted;
	Vector3d m_cmdVel;

	std::vector<geometry_msgs::Pose2D> 	m_tabMachines;
	std::vector<geometry_msgs::Pose2D> 	m_tabLandmarks;
	std::vector<geometry_msgs::Pose2D> 	m_scan;
	std::vector<int>					m_zones;

	ros::Time m_temps;
};

#endif