/**
 * \file 		refBoxComm.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-03
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef REFBOXCOMM_H_
#define REFBOXCOMM_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "comm_msg/GameState.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ReportMachine.h"

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/Team.pb.h>

#include "dispatch.h"
#include "sendScheduler.h"
#include "refBoxUtils.h"

class RefBoxTransport;
class RefBoxComm;

typedef google::protobuf::Message protoMsg;
typedef StaticDispatch<protoMsg, bool, std::function<bool(google::protobuf::Message&)>> MessageDispatcher;

class RefBoxComm
{
public:
    RefBoxComm(std::string teamName, std::string teamColor, std::string robotName, int robotNumber);
    virtual ~RefBoxComm();

    void setTransport(std::shared_ptr<RefBoxTransport> &refBoxTr);
private:
    ros::NodeHandle m_nh;

    ros::Publisher m_gameState_pub;
    ros::Publisher m_explorationInfo_pub;
    ros::Publisher m_machineReportInfo_pub;
    ros::Publisher m_orderInfo_pub;
    ros::Publisher m_machineInfo_pub;

    ros::ServiceServer m_reportMachineService;

    ros::Subscriber m_pose_sub;

    struct Status
    {
        Status():seq(0){}
        unsigned long seq;
        llsf_msgs::Team teamColor;
        llsf_msgs::Pose2D robotPose;
	    std::string teamName;
	    std::string robotName;
	    unsigned int robotNumber;

	    llsf_msgs::MachineReport machineToReport;
	    GamePhase gamePhase;

	    comm_msg::GameState gameState;
    } m_status;

    std::shared_ptr<RefBoxTransport> m_transport;
    std::shared_ptr<MessageDispatcher> m_dispatcher;

    SendScheduler m_sendScheduler;

    //ros callback
    void PoseCallback(const nav_msgs::Odometry &odom);

    //ros services
    bool ReportMachineSrv(comm_msg::ReportMachine::Request  &req,
                          comm_msg::ReportMachine::Response &res);


    //dispatcher receive handler
    bool fireBeaconSignal(protoMsg &m);
    bool fireGameState(protoMsg &m);
    bool fireOrderInfo(protoMsg &m);
    bool fireVersionInfo(protoMsg &m);
    bool fireExplorationInfo(protoMsg &m);
    bool fireMachineInfo(protoMsg &m);
    bool fireMachineReportInfo(protoMsg &m);
    bool fireRobotInfo(protoMsg &m);
    bool fireUnknown(protoMsg &m);

    //send callBack
    bool sendBeaconSignalCB(protoMsg &m);
    bool sendMachineReportCB(protoMsg &m);

};

#endif /* REFBOXCOMM_H_ */
