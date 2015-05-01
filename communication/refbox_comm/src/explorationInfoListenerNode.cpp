
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include "comm_msg/ExplorationInfo.h"

void eiCallback(const comm_msg::ExplorationInfo &ei)
{
    for(int idx=0; idx < ei.signals.size(); idx++)
    {
        ROS_INFO("signal %d : %s", idx, ei.signals[idx].type.c_str());

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorationInfoListener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/refBoxComm/ExplorationInfo", 1000, eiCallback);

    ros::Rate loop_rate(50);


    ros::spin();


    return 0;
}
