/**
 * \file         path_tracker_node.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-17
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */


#include <ros/ros.h>
#include "path_tracker/PathTracking.h"
#include "path_tracker/SwitchModeBehavior.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;

    //std::shared_ptr<SwitchModeBehavior> behavior(new SwitchModeBehavior());
    //PathTracking pathTracking("navigation/trackPath", behavior);


    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
