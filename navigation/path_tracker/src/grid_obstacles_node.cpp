#include "ros/ros.h"

#include "dataLaser.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GridObstacles");
    ros::NodeHandle nh;
    ros::spinOnce();
    DataLaser datalaser;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {       
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
