/**
 * \file        RobotPoseSubscriber.h
 * \class       RobotPoseSubscriber
 * \brief       classe pour s'abonner au Topic de la pose robot
 * \author      DANEL Thomas (th.danel@gmail.com)
 *              Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */

#ifndef COMMON_UTILS_ROBOT_POSE_SUBSCRIBER_H_
#define COMMON_UTILS_ROBOT_POSE_SUBSCRIBER_H_

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace common_utils {

class RobotPoseSubscriber
{
public:
    /* Constructeur */
    RobotPoseSubscriber(std::string destFrame_id = "map");

    /* Destructeur */
    virtual  ~RobotPoseSubscriber();

    /* Méthodes */
    void odomCallback(const nav_msgs::Odometry &msg);

    const geometry_msgs::Pose &getPose()
    {
        return m_pose;
    }
    const geometry_msgs::Pose2D &getPose2D()
    {
        return m_pose2D;
    }
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    tf::TransformListener m_tfListener;

    geometry_msgs::Pose m_pose;
    geometry_msgs::Pose2D m_pose2D;
    std::string m_destFrame_id;
};

} // namespace common_utils

#endif /* COMMON_UTILS_ROBOT_POSE_SUBSCRIBER_H_ */
