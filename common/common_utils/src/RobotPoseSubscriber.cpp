/**
 * \file        RobotPoseSubscriber.h
 * \class       RobotPoseSubscriber
 * \brief       classe pour s'abonner au Topic de la pose robot
 * \author      DANEL Thomas (th.danel@gmail.com)
 *              Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */
#include "common_utils/RobotPoseSubscriber.h"

namespace common_utils {

RobotPoseSubscriber::RobotPoseSubscriber(std::string destFrame_id):
    m_tfListener(m_nh, ros::Duration(5.0)),
    m_destFrame_id(destFrame_id)
{
    m_sub = m_nh.subscribe("hardware/odom",1,&RobotPoseSubscriber::odomCallback, this);
}

RobotPoseSubscriber::~RobotPoseSubscriber()
{

}

void RobotPoseSubscriber::odomCallback(const nav_msgs::Odometry &msg)
{

    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;
    poseIn.header = msg.header;
    poseIn.pose = msg.pose.pose;

    if(!m_tfListener.waitForTransform(
            poseIn.header.frame_id,
            m_destFrame_id,
            poseIn.header.stamp,
            ros::Duration(1.0)))
    {
        ROS_ERROR("Unable to get Transform");
        return;
    }
    m_tfListener.transformPose(m_destFrame_id, poseIn, poseOut);
    m_pose = poseOut.pose;

    m_pose2D.x = m_pose.position.x;
    m_pose2D.y = m_pose.position.y;
    m_pose2D.theta = tf::getYaw(m_pose.orientation);
}

} // namespace common_utils
