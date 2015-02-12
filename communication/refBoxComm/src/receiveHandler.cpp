/**
 * \file 		receiveHandler.cpp
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <stdio.h>
#include <iostream>

#include "receiveHandler.h"

using namespace llsf_msgs;

bool fireBeaconSignal(protoMsg &m)
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


bool fireGameState(protoMsg &m)
{
	GameState &gs = dynamic_cast<GameState&>(m);
	
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


/*
        switch (gs.phase())
        {
        default:
        case llsf_msgs::GameState::PRE_GAME:
            m_gamePhase = PHASE_PRE_GAME;
            break;
        case llsf_msgs::GameState::SETUP:
            m_gamePhase = PHASE_SETUP;
            break;
        case llsf_msgs::GameState::EXPLORATION:
            m_gamePhase = PHASE_EXPLORATION;
            break;
        case llsf_msgs::GameState::PRODUCTION:
            m_gamePhase = PHASE_PRODUCTION;
            break;
        case llsf_msgs::GameState::POST_GAME:
            m_gamePhase = PHASE_POST_GAME;
            break;
        }

*/

	return true;
}


bool fireOrderInfo(protoMsg &m)
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


bool fireVersionInfo(protoMsg &m)
{
	VersionInfo &vi = dynamic_cast<VersionInfo&>(m);
	
    printf("VersionInfo received: %s\n", vi.version_string().c_str());
    
	return true;
}

bool fireExplorationInfo(protoMsg &m)
{
	ExplorationInfo &ei = dynamic_cast<ExplorationInfo&>(m);
	
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
    
	return true;
}


bool fireMachineInfo(protoMsg &m)
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


bool fireMachineReportInfo(protoMsg &m)
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

bool fireRobotInfo(protoMsg &m)
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
	
bool fireUnknown(protoMsg &m)
{
    std::cout << "Message not yet handled" << std::endl;
}


