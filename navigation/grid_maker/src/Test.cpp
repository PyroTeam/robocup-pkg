#include "Test.hpp"

/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher Pose_Machines_Pub = n.advertise<deplacement_msg::Landmarks>("/landmarks", 1000);
    ROS_INFO("Ready to send poses machines");
    int xA,yA;
    ros::Rate loop_rate(1);
    float a = 0;
    while(ros::ok())
    {
        deplacement_msg::Landmarks machines_msgs;
        geometry_msgs::Pose2D tab;
        // - Left Side
        // // Zone 21
        // tab.x=-5;
        // tab.y=1;
        // tab.theta=0;
        // machines_msgs.landmarks.push_back(tab);
        // // Zone 23
        // tab.x=-4.5;
        // tab.y=3+0.6;
        // tab.theta=0;
        // machines_msgs.landmarks.push_back(tab);
        // // Zone 24
        // tab.x=-4.85;
        // tab.y=4.5+0.4;
        // tab.theta=-M_PI/4;
        // machines_msgs.landmarks.push_back(tab);

        // // Zone 18
        // tab.x=-3.5;
        // tab.y=1.5+0.75;
        // tab.theta=M_PI/2;
        // machines_msgs.landmarks.push_back(tab);

        // // Zone 14
        // tab.x=-1;
        // tab.y=1.5+0.75;
        // tab.theta=-M_PI/4;
        // machines_msgs.landmarks.push_back(tab);
        // Zone 16
        tab.x=-0.55;
        tab.y=5.35;
        tab.theta=M_PI/2;
        machines_msgs.landmarks.push_back(tab);


        // - Right Side
        // // Zone 21
        // tab.x=5;
        // tab.y=1;
        // tab.theta=0;
        // machines_msgs.landmarks.push_back(tab);
        // // Zone 23
        // tab.x=4.5;
        // tab.y=3+0.6;
        // tab.theta=0;
        // machines_msgs.landmarks.push_back(tab);
        // // Zone 24
        // tab.x=4.85;
        // tab.y=4.5+0.4;
        // tab.theta=+M_PI/4;
        // machines_msgs.landmarks.push_back(tab);

        // // Zone 18
        // tab.x=3.5;
        // tab.y=1.5+0.75;
        // tab.theta=-M_PI/2;
        // machines_msgs.landmarks.push_back(tab);

        // // Zone 14
        // tab.x=1;
        // tab.y=1.5+0.75;
        // tab.theta=+M_PI/4;
        // machines_msgs.landmarks.push_back(tab);
        // Zone 16
        tab.x=0.55;
        tab.y=5.35;
        tab.theta=M_PI/2;
        machines_msgs.landmarks.push_back(tab);

        Pose_Machines_Pub.publish(machines_msgs);
     	ros::spinOnce();
    	loop_rate.sleep();
 	}
    return 0;
}
