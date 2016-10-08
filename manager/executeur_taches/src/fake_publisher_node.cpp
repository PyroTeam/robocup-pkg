/* Fake Localisation Publisher Node */

#include "ros/ros.h"
#include "manager_msg/Landmarks.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "talker");

	ROS_INFO("Starting publishing ");

	ros::NodeHandle n;

	ros::Publisher landmarks_pub = n.advertise<manager_msg::Landmarks>("objectDetection/landmarks", 1000);

	ros::Rate loop_rate(1);

	int count = 0;
	while (ros::ok())
	{
		ROS_INFO("testing");
		std::vector<geometry_msgs::Pose2D> msg;

		while(count < 100000)
		{
			geometry_msgs::Pose2D pose;

			pose.x = count;
			pose.y = count;
			pose.theta = count;
			msg.push_back(pose);
			++count;
			ROS_INFO("count : %d", count);
			sleep(1);
		}

		manager_msg::Landmarks pub;
		pub.landmarks = msg;

		landmarks_pub.publish(pub);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}