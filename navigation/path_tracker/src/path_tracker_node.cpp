#include "ros/ros.h"
#include "path_tracker/trackPathAction.h"
#include "path_tracker/map.h"

nav_msgs::OccupancyGrid gridMap;
bool callBack = false;

void gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    ROS_INFO("CallBack");
    gridMap = grid;
    callBack = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PathTracker");
    ros::NodeHandle nh;
    ros::spinOnce();
    //TrackPathAction pathTrack("/trackPath");

    DataLaser datalaser;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {       
        sensor_msgs::LaserScan scan;
        datalaser.recoverDataLaser(scan);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
