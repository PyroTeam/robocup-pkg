#include "ros/ros.h"
#include "std_msgs/String.h"
#include "navigation_manager/moveToPose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Navigation_manager");
    ros::NodeHandle nh;
    MoveToPose dplct("/moveToPose");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
