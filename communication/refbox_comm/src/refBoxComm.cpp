/**
 * \file 		refBoxComm.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-03
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "refBoxComm.h"
#include "refBoxTransport.h"
#include "dispatch.h"
#include "refBoxUtils.h"

using namespace llsf_msgs;




RefBoxComm::RefBoxComm(std::string teamName, std::string teamColor, std::string robotName, int robotNumber)
{
    //configuration
    m_status.teamName = teamName;
    m_status.robotName = robotName;
    m_status.robotNumber = robotNumber;
  	m_status.teamColor = (teamColor == "cyan") ? CYAN : MAGENTA;
	m_status.gamePhase = PHASE_PRE_GAME;

    //initialisation des composants ROS
    m_gameState_pub = m_nh.advertise<comm_msg::GameState>("/refBoxComm/GameState", 1000);
    m_explorationInfo_pub = m_nh.advertise<comm_msg::ExplorationInfo>("/refBoxComm/ExplorationInfo", 1000);

    m_reportMachineService = m_nh.advertiseService("/refBoxComm/ReportMachine", &RefBoxComm::ReportMachineSrv, this);

    m_pose_sub = m_nh.subscribe("/odom", 1000, &RefBoxComm::PoseCallback, this);

    //lancement de l'ordonnanceur de messages
    m_sendScheduler.spawn();

    //ajout du message periodique BeaconSignal
    //uniquement si on est pas sur le PC du manager
    if (m_status.robotName != "manager" && m_status.robotNumber != 0)
    {
        std::shared_ptr<BeaconSignal> bm(new BeaconSignal());

        bm->set_number(m_status.robotNumber);
        bm->set_peer_name(m_status.robotName);
        bm->set_team_name(m_status.teamName);
        bm->set_team_color(m_status.teamColor);
        RefBoxMessage beaconMessage(bm, RefBoxMessage::PERIODIC, 1000.0);
        beaconMessage.setCallBack(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::sendBeaconSignalCB, this, _1)));
        m_sendScheduler.push(beaconMessage);
    }
}

RefBoxComm::~RefBoxComm()
{

}


void RefBoxComm::setTransport(std::shared_ptr<RefBoxTransport> &refBoxTr)
{
    m_transport = refBoxTr;

    //initialisation du dispatcher des messages reçus
    m_dispatcher = std::make_shared<MessageDispatcher>();
    m_dispatcher->Add<BeaconSignal>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireBeaconSignal, this, _1)));
    m_dispatcher->Add<OrderInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireOrderInfo, this, _1)));
    m_dispatcher->Add<GameState>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireGameState, this, _1)));
    m_dispatcher->Add<VersionInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireVersionInfo, this, _1)));
    m_dispatcher->Add<ExplorationInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireExplorationInfo, this, _1)));
    m_dispatcher->Add<MachineInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireMachineInfo, this, _1)));
    m_dispatcher->Add<MachineReportInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireMachineReportInfo, this, _1)));
    m_dispatcher->Add<RobotInfo>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireRobotInfo, this, _1)));
    m_dispatcher->Add<Team>(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::fireUnknown, this, _1)));

    m_transport->setReceiveDispatcher(m_dispatcher);

    m_sendScheduler.setTransport(m_transport);
}



void get_actualTime(unsigned long &sec, unsigned long &nsec)
{
    struct timespec actualTime;
    //TODO modifier avec std::chrono
    clock_gettime(CLOCK_REALTIME, &actualTime);

    sec = actualTime.tv_sec;
    nsec = actualTime.tv_nsec;
}

void RefBoxComm::PoseCallback(const nav_msgs::Odometry &odom)
{
    //ROS_INFO("Robot pose (%.3f, %.3f, %.3f)", pose.x, pose.y, pose.theta);
    //m_transport->set_pose(odom.pose.pose.position.x,
    //                 odom.pose.pose.position.y,
    //                 tf::getYaw(odom.pose.pose.orientation));
    m_status.robotPose.set_x(odom.pose.pose.position.x);
    m_status.robotPose.set_y(odom.pose.pose.position.y);
    m_status.robotPose.set_ori(tf::getYaw(odom.pose.pose.orientation));

    unsigned long sec = 0;
    unsigned long nsec = 0;
    get_actualTime(sec, nsec);

    Time *pose_time = m_status.robotPose.mutable_timestamp();
    pose_time->set_sec(static_cast<google::protobuf::int64>(sec));
    pose_time->set_nsec(static_cast<google::protobuf::int64>(nsec));

}

bool RefBoxComm::ReportMachineSrv(comm_msg::ReportMachine::Request  &req,
                                  comm_msg::ReportMachine::Response &res)
{
    ROS_INFO("Reporting the machine %s of type %s", req.name.c_str(), req.type.c_str());
    // m_transport->add_machine(req.name, req.type);

    bool addMessage = false;
    if ( m_status.machineToReport.machines_size()==0)
    {
        addMessage = true;
    }


    m_status.machineToReport.set_team_color(m_status.teamColor);


    llsf_msgs::MachineReportEntry *entry = m_status.machineToReport.add_machines();
    entry->set_name(req.name);
    entry->set_type(req.type);
    entry->set_zone(llsf_msgs::Zone(req.zone));

    if (addMessage)
    {
        std::shared_ptr<MachineReport> mr(new MachineReport());

        *mr = m_status.machineToReport;
        RefBoxMessage machineToReportMessage(mr, RefBoxMessage::PERIODIC, 1000.0);
        machineToReportMessage.setCallBack(std::function<bool(google::protobuf::Message&)>(boost::bind(&RefBoxComm::sendMachineReportCB, this, _1)));
        m_sendScheduler.push(machineToReportMessage);
    }


    return true;
}

//send CallBack functions
bool RefBoxComm::sendBeaconSignalCB(protoMsg &m)
{
    BeaconSignal &bs = dynamic_cast<BeaconSignal&>(m);

    Time *time = bs.mutable_time();
    unsigned long sec = 0;
    unsigned long nsec = 0;
    get_actualTime(sec, nsec);
    time->set_sec(static_cast<google::protobuf::int64>(sec));
    time->set_nsec(static_cast<google::protobuf::int64>(nsec));

    bs.set_seq(++m_status.seq);

    Pose2D *pose = bs.mutable_pose();
    pose->set_x(m_status.robotPose.x());
    pose->set_y(m_status.robotPose.y());
    pose->set_ori(m_status.robotPose.ori());

    Time *pose_time = pose->mutable_timestamp();
    Time *status_pose_time = m_status.robotPose.mutable_timestamp();
    pose_time->set_sec(status_pose_time->sec());
    pose_time->set_nsec(status_pose_time->nsec());

    return true;
}

bool RefBoxComm::sendMachineReportCB(protoMsg &m)
{
    MachineReport &mr = dynamic_cast<MachineReport&>(m);

    if(m_status.gameState.phase == comm_msg::GameState::EXPLORATION)
    {
        mr = m_status.machineToReport;
        return true;
    }
    else
    {
        //clean machineToReport
        m_status.machineToReport.mutable_machines()->Clear();
        return false;
    }

}


//receive Handlers
bool RefBoxComm::fireBeaconSignal(protoMsg &m)
{
	BeaconSignal &bs = dynamic_cast<BeaconSignal&>(m);

#if __WORDSIZE == 64
    printf("Detected robot: %u %s:%s (seq %lu)\n",
#else
    printf("Detected robot: %u %s:%s (seq %llu)\n",
#endif
         bs.number(), bs.team_name().c_str(), bs.peer_name().c_str(), bs.seq());
	return true;
}


bool RefBoxComm::fireGameState(protoMsg &m)
{
	GameState &gs = dynamic_cast<GameState&>(m);

/*
    int hour = gs.game_time().sec() / 3600;
    int min  = (gs.game_time().sec() - hour * 3600) / 60;
    int sec  = gs.game_time().sec() - hour * 3600 - min * 60;

#if __WORDSIZE == 64
    printf("GameState received:  %02i:%02i:%02i.%02ld  %s %s  %u:%u points, %s vs. %s\n",
#else
    printf("GameState received:  %02i:%02i:%02i.%02lld  %s %s  %u:%u points, %s vs. %s\n",
#endif
    hour, min, sec, gs.game_time().nsec() / 1000000,
    llsf_msgs::GameState::Phase_Name(gs.phase()).c_str(),
    llsf_msgs::GameState::State_Name(gs.state()).c_str(),
    gs.points_cyan(), gs.points_magenta(),
    gs.team_cyan().c_str(), gs.team_magenta().c_str());
*/

    //convert information into ROS type
    m_status.gameState = llsf2ros_gameState(gs, m_status.teamColor);
    //and publish
    m_gameState_pub.publish(m_status.gameState);

	return true;
}


bool RefBoxComm::fireOrderInfo(protoMsg &m)
{
	OrderInfo &oi = dynamic_cast<OrderInfo&>(m);

    printf("Order Info received:\n");
    for (int i = 0; i < oi.orders_size(); ++i)
    {
        const llsf_msgs::Order &o = oi.orders(i);
        unsigned int begin_min = o.delivery_period_begin() / 60;
        unsigned int begin_sec = o.delivery_period_begin() - begin_min * 60;
        unsigned int end_min = o.delivery_period_end() / 60;
        unsigned int end_sec = o.delivery_period_end() - end_min * 60;

        printf("  %u: %u/%u of %s from %02u:%02u to %02u:%02u at gate %s\n", o.id(),
        o.quantity_delivered(), o.quantity_requested(),
        llsf_msgs::Order::ProductType_Name(o.product()).c_str(),
        begin_min, begin_sec, end_min, end_sec,
        llsf_msgs::Order::DeliveryGate_Name(o.delivery_gate()).c_str());
    }

	return true;
}


bool RefBoxComm::fireVersionInfo(protoMsg &m)
{
	VersionInfo &vi = dynamic_cast<VersionInfo&>(m);

    //printf("VersionInfo received: %s\n", vi.version_string().c_str());

	return true;
}

bool RefBoxComm::fireExplorationInfo(protoMsg &m)
{
	ExplorationInfo &ei = dynamic_cast<ExplorationInfo&>(m);

	/*
    printf("ExplorationInfo received:\n");
    for (int i = 0; i < ei.signals_size(); ++i)
    {
        const ExplorationSignal &es = ei.signals(i);
        printf("  Machine type %s assignment:", es.type().c_str());
        for (int j = 0; j < es.lights_size(); ++j)
        {
            const LightSpec &lspec = es.lights(j);
            printf(" %s=%s", LightColor_Name(lspec.color()).c_str(),
                    LightState_Name(lspec.state()).c_str());
        }
        printf("\n");
    }
    printf("  --\n");
    for (int i = 0; i < ei.machines_size(); ++i)
    {
        const ExplorationMachine &em = ei.machines(i);
        printf("  Machine %s at (%f, %f, %f)\n", em.name().c_str(),
                em.pose().x(), em.pose().y(), em.pose().ori());
    }
    */

    //convert information into ROS type
    comm_msg::ExplorationInfo explorationInfo;
    explorationInfo = llsf2ros_explorationInfo(ei, m_status.teamColor);

    //and publish
    m_explorationInfo_pub.publish(explorationInfo);

	return true;
}


bool RefBoxComm::fireMachineInfo(protoMsg &m)
{
	MachineInfo &mi = dynamic_cast<MachineInfo&>(m);

    printf("MachineInfo received:\n");
    for (int i = 0; i < mi.machines_size(); ++i)
    {
        const Machine &m = mi.machines(i);
        const Pose2D &p = m.pose();
        printf("  %-3s|%2s|%s @ (%f, %f, %f)\n",
                m.name().c_str(), m.type().substr(0, 2).c_str(),
        Team_Name(m.team_color()).substr(0, 2).c_str(),
                p.x(), p.y(), p.ori());
    }

	return true;
}


bool RefBoxComm::fireMachineReportInfo(protoMsg &m)
{
	MachineReportInfo &mri = dynamic_cast<MachineReportInfo&>(m);

    printf("MachineReportInfo received:\n");
    if (mri.reported_machines_size() > 0)
    {
        printf("  Reported machines:");
        for (int i = 0; i < mri.reported_machines_size(); ++i)
        {
            printf(" %s", mri.reported_machines(i).c_str());
        }
        printf("\n");
    }
    else
    {
        printf("  no machines reported, yet\n");
    }

	return true;
}

bool RefBoxComm::fireRobotInfo(protoMsg &m)
{
	RobotInfo &ri = dynamic_cast<RobotInfo&>(m);

    printf("Robot Info received:\n");
    for (int i = 0; i < ri.robots_size(); ++i)
    {
        const llsf_msgs::Robot &r = ri.robots(i);
        const llsf_msgs::Time &time = r.last_seen();
/*
        boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
        boost::posix_time::ptime last_seen =
        boost::posix_time::from_time_t(time.sec())
            + boost::posix_time::nanoseconds(time.nsec());


        boost::posix_time::time_duration const last_seen_ago_td = now - last_seen;
        float last_seen_ago = last_seen_ago_td.total_milliseconds() / 1000.f;
*/
        float last_seen_ago = 0;
        printf("  %u %s/%s @ %s: state %s, last seen %f sec ago  Maint cyc: %u  rem: %f\n",
                r.number(), r.name().c_str(), r.team().c_str(), r.host().c_str(),
        llsf_msgs::RobotState_Name(r.state()).substr(0,3).c_str(),
                last_seen_ago, r.maintenance_cycles(), r.maintenance_time_remaining());
    }


	return true;
}

bool RefBoxComm::fireUnknown(protoMsg &m)
{
    std::cout << "Message not yet handled" << std::endl;
}
