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

        // -- Tests version open German

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
        // tab.x=-0.55;
        // tab.y=5.35;
        // tab.theta=M_PI/2;
        // machines_msgs.landmarks.push_back(tab);


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
        // tab.x=0.55;
        // tab.y=5.35;
        // tab.theta=M_PI/2;
        // machines_msgs.landmarks.push_back(tab);

        // -- Tests version Thomas

        // - Left Side
        // Zone 21
        tab.x=-5.080;
        tab.y=0.997;
        tab.theta=2.086;
        machines_msgs.landmarks.push_back(tab);
        // Zone 23
        tab.x=-4.599;
        tab.y=3.619;
        tab.theta=2.122;
        machines_msgs.landmarks.push_back(tab);
        // Zone 24
        tab.x=-2.543;
        tab.y=0.713;
        tab.theta=0.598;
        machines_msgs.landmarks.push_back(tab);

        // Zone 18
        tab.x=-3.222;
        tab.y=4.868;
        tab.theta=1.608;
        machines_msgs.landmarks.push_back(tab);

        // Zone 14
        tab.x=-1.097;
        tab.y=2.177;
        tab.theta=0.856;
        machines_msgs.landmarks.push_back(tab);
        // Zone 16
        tab.x=-0.539;
        tab.y=4.965;
        tab.theta=1.673;
        machines_msgs.landmarks.push_back(tab);


        // - Right Side
        // Zone 21
        tab.x=3.174;
        tab.y=5.167;
        tab.theta=0.797;
        machines_msgs.landmarks.push_back(tab);
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
        // tab.x=0.55;
        // tab.y=5.35;
        // tab.theta=M_PI/2;
        // machines_msgs.landmarks.push_back(tab);

        Pose_Machines_Pub.publish(machines_msgs);
     	ros::spinOnce();
    	loop_rate.sleep();
 	}
    return 0;
}