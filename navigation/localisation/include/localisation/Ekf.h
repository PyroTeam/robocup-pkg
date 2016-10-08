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
#include "LandmarksDetectionSubscriber.h"

using namespace Eigen;

class EKF{
public:
    EKF();

    bool initOdom();
    void addMachine(const Vector3d &machine);
    int checkStateVector(const Vector3d &machine);
    MatrixXd buildH2(int size, int i);

    void predict();
    void run();
    void correctOnce(int i);
    void correct();

    void setTF(tf::TransformListener *tf_listener){m_tf_listener = tf_listener;}
    geometry_msgs::Pose2D getRobot();
    std::vector<geometry_msgs::Pose2D> getLandmarks();
private:
    ros::Time m_initTime;
    ros::Time m_time;
    tf::TransformListener *m_tf_listener;
    common_utils::RobotPoseSubscriber m_poseSub;
    LandmarksDetectionSubscriber m_landmarksSub;

    MatrixXd m_predictedP;
    MatrixXd m_P;
    VectorXd m_predictedState;
    VectorXd m_state;

    bool 	m_begin;
};

#endif
