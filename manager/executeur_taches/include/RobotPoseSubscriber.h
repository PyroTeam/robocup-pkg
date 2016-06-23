/**
 * \file        RobotPoseSubscriber.h
 * \class       RobotPoseSubscriber
 * \brief       classe pour s'abonner au Topic de la pose robot
 * \author      DANEL Thomas (th.danel@gmail.com)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */

#ifndef ROBOT_POSE_SUBSCRIBER_H
#define ROBOT_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class RobotPoseSubscriber
{
	public:
    /* Constructeur */
    RobotPoseSubscriber();

    /* Destructeur */
    virtual  ~RobotPoseSubscriber();

    /* Méthodes */
    void odomCallback(const nav_msgs::Odometry &msg);

	private:
		ros::Subscriber m_sub;
    geometry_msgs::Pose2D m_pose;
};
#endif
