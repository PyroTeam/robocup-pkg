#include "RobotPoseSubscriber.h"

RobotPoseSubscriber::RobotPoseSubscriber()
{
	ros::NodeHandle n;
	m_sub = n.subscribe("hardware/odom",1,&RobotPoseSubscriber::odomCallback, this);
}

RobotPoseSubscriber::~RobotPoseSubscriber()
{

}

void RobotPoseSubscriber::odomCallback(const nav_msgs::Odometry &msg)
{
  m_pose.x = msg.pose.pose.position.x;
  m_pose.y = msg.pose.pose.position.y;
  m_pose.theta = tf::getYaw(msg.pose.pose.orientation);
}
