#include "ros/ros.h"
#include "path_tracker/trackPathAction.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PathTracker");
    ros::NodeHandle nh;
    //TrackPathAction pathTrack("/trackPath");
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
