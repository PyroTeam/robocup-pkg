#include "tf_broadcaster/tf_broadcaster.h"

void poseCallback(const nav_msgs::Odometry &odom)
{
    static tf::TransformBroadcaster mapToOdom;
    static tf::TransformBroadcaster odomToBaseLink;
    static tf::TransformBroadcaster baseLinkToLaserLink;

    // Map to odom
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    mapToOdom.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "odom"));

    // Odom to Base Link
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
    transform.setRotation(q);
    odomToBaseLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "odom", "base_link"));

    // Base Link to Laser Link
    static double laser_link_x = 0;
    ros::NodeHandle nh;
    nh.param<double>("/laser_link_x", laser_link_x, 0.10);
    transform.setOrigin(tf::Vector3(laser_link_x, 0.0, 0.232));
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    baseLinkToLaserLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "base_link", "laser_link"));
}