

#include "ros/ros.h"

#include "Activity.pb.h"
#include "Beacon.pb.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "InterRobotComm");
    ros::NodeHandle nh;

    //chargement de la configuration
    int robotNumber;
    nh.param<int>("robotNumber", robotNumber, 0);


    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
