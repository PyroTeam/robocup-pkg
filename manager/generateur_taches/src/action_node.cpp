#include "action.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_node");
    
    Action action;
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
