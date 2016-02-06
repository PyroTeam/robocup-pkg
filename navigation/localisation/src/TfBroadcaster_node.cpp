#include "tf_broadcaster.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;

    ros::Rate r(50);

    // Subscribe to odom 
    ros::Subscriber sub_odom = n.subscribe("hardware/odom", 1000, &poseCallback);

    while(n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}