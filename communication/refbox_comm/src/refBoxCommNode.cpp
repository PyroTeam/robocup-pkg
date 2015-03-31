/**
 * \file 		refBoxCommNode.cpp
 *
 * \brief		Code source principal du noeuds de communication avec 
 *              la referee Box pour la competition robocup (Logistic League)
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2014-06-16
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#include "refBoxTransport.h"
#include "dispatch.h"
#include "sendScheduler.h"
#include "refBoxComm.h"
#include "refBoxUtils.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "comm_msg/GameState.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ReportMachine.h"
#include <tf/transform_datatypes.h>

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/Team.pb.h>

#include <sstream>
#include <string>


int main(int argc, char **argv)
{

    int loopFreq = 50;
    Team team_color;
    
    ros::init(argc, argv, "refBoxComm");
    ros::NodeHandle nh;
    
    //obtenir la configuration
    std::string teamName;
    std::string teamColor;
    std::string robotName;
    int robotNumber;
    nh.param<std::string>("teamName",teamName, "PyroTeam");
	nh.param<std::string>("robotName", robotName, "R0");
	nh.param<int>("robotNumber", robotNumber, 0);
	nh.param<std::string>("teamColor", teamColor, "cyan");
	

    std::shared_ptr<RefBoxTransport> refBoxTPtr(new RefBoxTransport());
    refBoxTPtr->init(teamColor);
    // team_color = refBoxTPtr->get_teamColor();

    RefBoxComm refBoxComm(teamName, teamColor, robotName, robotNumber);
    refBoxComm.setTransport(refBoxTPtr);

    ros::Rate loop_rate(loopFreq);

    while (ros::ok() && !refBoxTPtr->isExit() )
    {
        //refBoxTPtr->update();

        ros::spinOnce();    
        loop_rate.sleep();
    }

    return 0;
}

