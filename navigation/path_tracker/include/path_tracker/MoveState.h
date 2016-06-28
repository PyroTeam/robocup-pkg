/**
 * \file         MoveState.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-17
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_MOVESTATE_H_
#define PATH_TRACKER_MOVESTATE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

class MoveState
{
public:
    MoveState();
    virtual ~MoveState();

    void setCurrentSetpoint(geometry_msgs::Twist &t);
    const geometry_msgs::Twist &getPreviousSetpoint();

    const geometry_msgs::Pose &getPose();
    const geometry_msgs::Pose2D &getPose2D();
protected:

    geometry_msgs::Twist m_currentSetpoint;
    geometry_msgs::Twist m_previousSetpoint;

    nav_msgs::Odometry m_currentPose;
};

#endif /* PATH_TRACKER_MOVESTATE_H_ */
