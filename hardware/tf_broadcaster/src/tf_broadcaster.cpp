#include "tf_broadcaster/tf_broadcaster.h"

void poseCallback(const nav_msgs::Odometry &odom)
{
	static tf::TransformBroadcaster odomToBaseLink;
	static ros::NodeHandle nh;
  static ros::NodeHandle nhPriv("~");
	bool onlyOdomToBase;

	std::string tf_prefix;
	nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
	if (tf_prefix.size() != 0)
		tf_prefix += "/";

    if (nhPriv.hasParam("onlyOdomToBase"))
    {
        nhPriv.getParam("onlyOdomToBase", onlyOdomToBase);
    }
    else
    {
        onlyOdomToBase = false;
    }

	tf::Transform transform;
	tf::Quaternion q;

	// Odom to Base Link
	transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
	tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
	transform.setRotation(q);
	odomToBaseLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp
		, tf_prefix+"odom", tf_prefix+"base_link"));

	if (onlyOdomToBase)
	{
		return;
	}

	static tf::TransformBroadcaster mapToOdom;
	static tf::TransformBroadcaster baseLinkToLaserLink;
	static tf::TransformBroadcaster baseLinkToTowerCameraLink;
	static tf::TransformBroadcaster baseLinkToPlatformCameraLink;

	// Map to odom
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	q.setRPY(0.0, 0.0, 0.0);
	transform.setRotation(q);
	mapToOdom.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", tf_prefix+"odom"));

	// Base Link to Laser Link
	static double laser_link_x = 0;
	static double laser_link_y = 0;
	static double laser_link_z = 0;
	nh.param<double>("hardware/robotDescription/baseLink_to_laserLink/x", laser_link_x, 0.10+0.01);
	nh.param<double>("hardware/robotDescription/baseLink_to_laserLink/y", laser_link_y, 0.0);
	nh.param<double>("hardware/robotDescription/baseLink_to_laserLink/z", laser_link_z, 0.235+0.0175);
	transform.setOrigin(tf::Vector3(laser_link_x, laser_link_y, laser_link_z));
	q.setRPY(0.0, 0.0, 0.0);
	transform.setRotation(q);
	baseLinkToLaserLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp, tf_prefix+"base_link", tf_prefix+"laser_link"));

	// Base Link to Tower Camera Link
	static double tower_camera_link_x = 0;
	static double tower_camera_link_y = 0;
	static double tower_camera_link_z = 0;
	nh.param<double>("hardware/robotDescription/baseLink_to_towerCameraLink/x", tower_camera_link_x, 0.20);
	nh.param<double>("hardware/robotDescription/baseLink_to_towerCameraLink/y", tower_camera_link_y, 0.00);
	nh.param<double>("hardware/robotDescription/baseLink_to_towerCameraLink/z", tower_camera_link_z, 0.53);
	transform.setOrigin(tf::Vector3(tower_camera_link_x, tower_camera_link_y, tower_camera_link_z));
	q.setRPY(-1.57, 0, -1.57);
	transform.setRotation(q);
	baseLinkToTowerCameraLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp
		, tf_prefix+"base_link", tf_prefix+"tower_camera_link"));

	// Base Link to Platform Camera Link
	static double platform_camera_link_x = 0;
	static double platform_camera_link_y = 0;
	static double platform_camera_link_z = 0;
	nh.param<double>("hardware/robotDescription/baseLink_to_platformCameraLink/x", platform_camera_link_x, 0.0);
	nh.param<double>("hardware/robotDescription/baseLink_to_platformCameraLink/y", platform_camera_link_y, 0.00);
	nh.param<double>("hardware/robotDescription/baseLink_to_platformCameraLink/z", platform_camera_link_z, 0.90);
	transform.setOrigin(tf::Vector3(platform_camera_link_x, platform_camera_link_y, platform_camera_link_z));
	q.setRPY(-1.57, 0, -1.57);
	transform.setRotation(q);
	baseLinkToPlatformCameraLink.sendTransform(tf::StampedTransform(transform, odom.header.stamp
		, tf_prefix+"base_link", tf_prefix+"platform_camera_link"));
}
