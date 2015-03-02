/**
 * \file 		refBoxCommNode.cpp
 *
 * \brief		Code source principal du moeuds de communication avec 
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
#include "receiveHandler.h"

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

refBoxTransport refboxT;


bool Report(comm_msg::ReportMachine::Request  &req,
            comm_msg::ReportMachine::Response &res);

void PoseCallback(const nav_msgs::Odometry &odom);

comm_msg::GameState llsf2ros_gameState(llsf_msgs::GameState llsfGameState, Team team_color);
comm_msg::ExplorationInfo llsf2ros_explorationInfo(llsf_msgs::ExplorationInfo llsfExplorationInfo, Team team_color);




int main(int argc, char **argv)
{

    int loopFreq = 50;
    Team team_color;
    
    ros::init(argc, argv, "refBoxComm");

    std::shared_ptr<MessageDispatcher> disp(new MessageDispatcher());
    disp->Add<BeaconSignal>(&fireBeaconSignal);
    disp->Add<OrderInfo>(&fireOrderInfo);
    disp->Add<GameState>(&fireGameState);
    disp->Add<VersionInfo>(&fireVersionInfo);
    disp->Add<ExplorationInfo>(&fireExplorationInfo);
    disp->Add<MachineInfo>(&fireMachineInfo);
    disp->Add<MachineReportInfo>(&fireMachineReportInfo);
    disp->Add<RobotInfo>(&fireRobotInfo);
    disp->Add<Team>(&fireUnknown);
    
    refboxT.setReceiveDispatcher(disp);
    refboxT.init();
    team_color = refboxT.get_teamColor();

    ros::NodeHandle n;

    ros::Publisher GameState_pub = n.advertise<comm_msg::GameState>("/refBoxComm/GameState", 1000);
    ros::Publisher ExplorationInfo_pub = n.advertise<comm_msg::ExplorationInfo>("/refBoxComm/ExplorationInfo", 1000);
  
    ros::ServiceServer service = n.advertiseService("/refBoxComm/ReportMachine", Report);
  

    ros::Subscriber sub = n.subscribe("/odom", 1000, PoseCallback);
  
    ros::Rate loop_rate(loopFreq);


    int count = 0;
    while (ros::ok() && !refboxT.isExit() )
    {

        comm_msg::GameState gameState;
        comm_msg::ExplorationInfo explorationInfo;
        
        gameState = llsf2ros_gameState(refboxT.get_gameState(), team_color);
        explorationInfo = llsf2ros_explorationInfo(refboxT.get_explorationInfo(), team_color);
        //ROS_INFO("Game state");

        if (!(count%loopFreq))
        {
            refboxT.handle_timer();

            GameState_pub.publish(gameState);        
            
            if (refboxT.get_phase() == PHASE_EXPLORATION)
            {
                ExplorationInfo_pub.publish(explorationInfo);
            }
        }
    
        refboxT.update();
    
        ros::spinOnce();
    
        loop_rate.sleep();
        ++count;
    }

    return 0;
}



bool Report(comm_msg::ReportMachine::Request  &req,
            comm_msg::ReportMachine::Response &res)
{
    ROS_INFO("Reporting the machine %s of type %s", req.name.c_str(), req.type.c_str());
    refboxT.add_machine(req.name, req.type);
    return true;
}

void PoseCallback(const nav_msgs::Odometry &odom)
{
    //ROS_INFO("Robot pose (%.3f, %.3f, %.3f)", pose.x, pose.y, pose.theta);
    refboxT.set_pose(odom.pose.pose.position.x, 
                     odom.pose.pose.position.y, 
                     tf::getYaw(odom.pose.pose.orientation));
}

comm_msg::GameState llsf2ros_gameState(llsf_msgs::GameState llsfGameState, Team team_color)
{
    comm_msg::GameState rosGameState;
        
    switch (llsfGameState.state())
    {
    default:
    case llsf_msgs::GameState::INIT:
        rosGameState.state = comm_msg::GameState::INIT;
        break;
    case llsf_msgs::GameState::WAIT_START:
        rosGameState.state = comm_msg::GameState::WAIT_START;
        break;
    case llsf_msgs::GameState::RUNNING:
        rosGameState.state = comm_msg::GameState::RUNNING;
        break;
    case llsf_msgs::GameState::PAUSED:
        rosGameState.state = comm_msg::GameState::PAUSED;
        break;        
    }
    
    switch (llsfGameState.phase())
    {
    default:
    case llsf_msgs::GameState::PRE_GAME:
        rosGameState.phase = comm_msg::GameState::PRE_GAME;
        break;
    case llsf_msgs::GameState::SETUP:
        rosGameState.phase = comm_msg::GameState::SETUP;
        break;
    case llsf_msgs::GameState::EXPLORATION:
        rosGameState.phase = comm_msg::GameState::EXPLORATION;
        break;
    case llsf_msgs::GameState::PRODUCTION:
        rosGameState.phase = comm_msg::GameState::PRODUCTION;
        break;
    case llsf_msgs::GameState::POST_GAME:
        rosGameState.phase = comm_msg::GameState::POST_GAME;
        break;
    case llsf_msgs::GameState::OPEN_CHALLENGE:
        rosGameState.phase = comm_msg::GameState::OPEN_CHALLENGE;
        break;
    case llsf_msgs::GameState::NAVIGATION_CHALLENGE:
        rosGameState.phase = comm_msg::GameState::NAVIGATION_CHALLENGE;
        break;
    case llsf_msgs::GameState::WHACK_A_MOLE_CHALLENGE:
        rosGameState.phase = comm_msg::GameState::WHACK_A_MOLE_CHALLENGE;
        break;
    }
    
    rosGameState.points = ((team_color == CYAN) ? 
                llsfGameState.points_cyan() : llsfGameState.points_magenta());
    
    rosGameState.game_time.sec = llsfGameState.game_time().sec();
    rosGameState.game_time.nsec = llsfGameState.game_time().nsec();

    return rosGameState;
}

comm_msg::ExplorationInfo llsf2ros_explorationInfo(llsf_msgs::ExplorationInfo llsfExplorationInfo, Team team_color)
{
    comm_msg::ExplorationInfo rosExplorationInfo;


    for (int i = 0; i < llsfExplorationInfo.signals_size(); ++i) 
    {
        const ExplorationSignal &es = llsfExplorationInfo.signals(i);
        comm_msg::ExplorationSignal signal;
        for (int j = 0; j < es.lights_size(); ++j) 
        {
            const LightSpec &lspec = es.lights(j);
            comm_msg::LightSpec light;
            signal.type = es.type();

            light.color = lspec.color();
            light.state = lspec.state();
            signal.lights.push_back(light);
        }
        rosExplorationInfo.signals.push_back(signal);
    }

    for (int i = 0; i < llsfExplorationInfo.machines_size(); ++i) 
    {
        const ExplorationMachine &em = llsfExplorationInfo.machines(i);
        comm_msg::ExplorationMachine machine;
        
        machine.name = em.name();
        machine.pose.x = em.pose().x();
        machine.pose.y = em.pose().y();
        machine.pose.theta =em.pose().ori();
        machine.team_color = em.team_color();
        
        rosExplorationInfo.machines.push_back(machine);
    }

    return rosExplorationInfo;
}
