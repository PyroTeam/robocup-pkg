#ifndef EKF_H
#define EKF_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <string>

#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Robot.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"
#include "common_utils/RobotPoseSubscriber.h"

using namespace Eigen;

class EKF{
public:
    EKF();

    void set();
    void printAreas();

    //callbacks
    void odomCallback(const nav_msgs::Odometry& odom);
    void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines);

    void addMachine(const geometry_msgs::Pose2D &machine);

    int checkStateVector(const geometry_msgs::Pose2D &p);
    bool initOdom();
    bool test(int area);
    //bool isFarFromEverything(const geometry_msgs::Pose2D &p);

    void updateP(const MatrixXd &Pm, int i);
    void updatePprev(const MatrixXd &Pm, int i);
    void updateXmean(const VectorXd &tmp, int i);

    MatrixXd buildPm(int i);
    MatrixXd buildH(int i);
    MatrixXd buildH2(const geometry_msgs::Pose2D &p, int size, int i);

    void prediction();
    void correction(geometry_msgs::Pose2D p, int posMachineInTabMachines);

    VectorXd getXmean();
    VectorXd getXpredicted();

    std::vector<geometry_msgs::Pose2D> getTabMachines();
    std::vector<int> getAreas();

    void setArea(int i){m_areas.push_back(i);}
    void setTF(tf::TransformListener *tf_listener){m_tf_listener = tf_listener;}

    deplacement_msg::Robot getRobot(){return m_robot;}
private:
    deplacement_msg::Robot m_robot;

    common_utils::RobotPoseSubscriber m_poseSub;

    VectorXd m_xMean;
    MatrixXd m_P;
    MatrixXd m_P_prev;

    VectorXd m_xPredicted;

    std::vector<int> m_areas;
    std::vector<geometry_msgs::Pose2D> m_mps;

    ros::Time m_temps;
    tf::TransformListener *m_tf_listener;
    bool 	m_begin;
};

#endif
