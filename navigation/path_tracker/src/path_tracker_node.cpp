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
#include "path_tracker/BasicFollower.h"
#include "common_utils/controller/PidWithAntiWindUp.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;

    std::shared_ptr<common_utils::PidWithAntiWindUp> pid(new common_utils::PidWithAntiWindUp(10, 0, 0, 1/10.0));
    std::shared_ptr<BasicFollower> pathFollower(new BasicFollower(pid));
    std::shared_ptr<SwitchModeBehavior> behavior(new SwitchModeBehavior());
    behavior->setPathFollower(pathFollower);
    PathTracking pathTracking("navigation/trackPath", behavior);


    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
