#include "Test.hpp"


/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher Pose_Machines_Pub = n.advertise<deplacement_msg::Landmarks>("/machines", 1000);
    ROS_INFO("Ready to send poses machines");
    int xA,yA;
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
    deplacement_msg::Landmarks machines_msgs;
    geometry_msgs::Pose2D tab;
    tab.x=-2;
    tab.y=2;
    tab.theta=0;
    machines_msgs.landmarks.push_back(tab);
    tab.x=-5;
    tab.y=5;
    tab.theta=3;
    machines_msgs.landmarks.push_back(tab);
 	Pose_Machines_Pub.publish(machines_msgs);
 	ros::spinOnce();
	loop_rate.sleep();
 	}
    return 0;
}