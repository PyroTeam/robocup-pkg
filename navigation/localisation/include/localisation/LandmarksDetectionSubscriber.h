/**
 * \file        LandmarksDetectionSubscriber.h
 * \class       LandmarksDetectionSubscriber
 * \brief       classe pour s'abonner au Topic de la pose robot
 * \author      DANEL Thomas (th.danel@gmail.com)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */

#ifndef COMMON_UTILS_LANDMARKS_DETECTION_SUBSCRIBER_H_
#define COMMON_UTILS_LANDMARKS_DETECTION_SUBSCRIBER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "deplacement_msg/Landmarks.h"

class LandmarksDetectionSubscriber
{
public:
    LandmarksDetectionSubscriber();
    virtual  ~LandmarksDetectionSubscriber();

    void machinesCallback(const deplacement_msg::Landmarks& machines);
    void segmentsCallback(const deplacement_msg::Landmarks& segments);

    const std::vector<Eigen::Vector3d> &getMachines(){return m_machines;}
    const Eigen::Vector3d &getMachine(int i){return m_machines[i];}

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    tf::TransformListener m_tfListener;

    std::vector<Eigen::Vector3d> m_machines;
};

#endif
