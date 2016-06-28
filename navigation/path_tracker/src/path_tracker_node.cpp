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
#include "path_tracker/BasicAvoidance.h"
#include "common_utils/controller/PidWithAntiWindUp.h"
#include "common_utils/Parameter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;

    double loopFreq = 10;
    ros::Rate loopRate(loopFreq);//TODO parametrer


    Parameter pidVelKp(nh, "navigation/PathTracker/Following/pidVel/Kp", 0.1);
    Parameter pidVelKi(nh, "navigation/PathTracker/Following/pidVel/Ki", 0.0);
    Parameter pidVelKd(nh, "navigation/PathTracker/Following/pidVel/Kd", 0.0);
    Parameter pidVelLowLimit(nh, "navigation/PathTracker/Following/pidVel/lowLimit", -10.0);
    Parameter pidVelUpLimit(nh, "navigation/PathTracker/Following/pidVel/upLimit", 10.0);
    Parameter pidVelAntiWindUp(nh, "navigation/PathTracker/Following/pidVel/antiWindUp", 0.1);

    std::shared_ptr<common_utils::PidWithAntiWindUp> pidVel(
        new common_utils::PidWithAntiWindUp(
            pidVelKp(),
            pidVelKi(),
            pidVelKd(),
            1/loopFreq,
            pidVelLowLimit(),
            pidVelUpLimit(),
            pidVelAntiWindUp()));

    Parameter pidOriKp(nh, "navigation/PathTracker/Following/pidOri/Kp", 1.5);
    Parameter pidOriKi(nh, "navigation/PathTracker/Following/pidOri/Ki", 0.01);
    Parameter pidOriKd(nh, "navigation/PathTracker/Following/pidOri/Kd", 0.0);
    Parameter pidOriLowLimit(nh, "navigation/PathTracker/Following/pidOri/lowLimit", -1.0);
    Parameter pidOriUpLimit(nh, "navigation/PathTracker/Following/pidOri/upLimit", 1.0);
    Parameter pidOriAntiWindUp(nh, "navigation/PathTracker/Following/pidOri/antiWindUp", 0.1);


    std::shared_ptr<common_utils::PidWithAntiWindUp> pidOri(
        new common_utils::PidWithAntiWindUp(
            pidOriKp(),
            pidOriKi(),
            pidOriKd(),
            1/loopFreq,
            pidOriLowLimit(),
            pidOriUpLimit(),
            pidOriAntiWindUp()));

    std::shared_ptr<BasicFollower> pathFollower(new BasicFollower(pidVel, pidOri));
    Parameter speedRateLowLimit(nh, "navigation/PathTracker/Following/SpeedRateLimit/lowLimit", -0.20);
    Parameter speedRateUpLimit(nh, "navigation/PathTracker/Following/SpeedRateLimit/upLimit", 0.20);
    pathFollower->setSpeedRateLimits(speedRateLowLimit(), speedRateUpLimit(), 1/loopFreq);

    std::shared_ptr<BasicAvoidance> avoidObstacle(new BasicAvoidance());
    avoidObstacle->setThreshObstacle(99);

    std::shared_ptr<SwitchModeBehavior> behavior(new SwitchModeBehavior());
    behavior->setPathFollower(pathFollower);
    behavior->setAvoidObstacle(avoidObstacle);

    PathTracking pathTracking("navigation/trackPath", behavior);



    while(ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
