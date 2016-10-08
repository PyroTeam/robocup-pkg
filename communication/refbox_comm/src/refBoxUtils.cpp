/**
 * \file 		refBoxUtils.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-05
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "refBoxUtils.h"

comm_msg::GameState llsf2ros_gameState(const llsf_msgs::GameState &llsfGameState, llsf_msgs::Team team_color)
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
    }

    rosGameState.points = ((team_color == llsf_msgs::Team::CYAN) ?
                llsfGameState.points_cyan() : llsfGameState.points_magenta());

    rosGameState.game_time.sec = llsfGameState.game_time().sec();
    rosGameState.game_time.nsec = llsfGameState.game_time().nsec();

    return rosGameState;
}


comm_msg::ExplorationInfo llsf2ros_explorationInfo(const llsf_msgs::ExplorationInfo &llsfExplorationInfo, llsf_msgs::Team team_color)
{
    comm_msg::ExplorationInfo rosExplorationInfo;


    for (int i = 0; i < llsfExplorationInfo.signals_size(); ++i)
    {
        const llsf_msgs::ExplorationSignal &es = llsfExplorationInfo.signals(i);
        comm_msg::ExplorationSignal signal;
        for (int j = 0; j < es.lights_size(); ++j)
        {
            const llsf_msgs::LightSpec &lspec = es.lights(j);
            comm_msg::LightSpec light;
            signal.type = es.type();

            light.color = lspec.color();
            light.state = lspec.state();
            signal.lights.push_back(light);
        }
        rosExplorationInfo.signals.push_back(signal);
    }

    for (int i = 0; i < llsfExplorationInfo.zones_size(); ++i)
    {
        const llsf_msgs::ExplorationZone &ez = llsfExplorationInfo.zones(i);
        comm_msg::ExplorationZone eZone;

        eZone.zone = uint8_t(ez.zone());
        eZone.team_color = uint8_t(ez.team_color());

        rosExplorationInfo.zones.push_back(eZone);
    }

    return rosExplorationInfo;
}

comm_msg::MachineInfo llsf2ros_machineInfo(const llsf_msgs::MachineInfo &llsfMachineInfo)
{
    comm_msg::MachineInfo rosMachineInfo;

    rosMachineInfo.team_color = uint8_t(llsfMachineInfo.team_color());

    for(int i=0; i< llsfMachineInfo.machines_size(); ++i)
    {
        const llsf_msgs::Machine &machine = llsfMachineInfo.machines(i);
        comm_msg::Machine rosMachine;

        rosMachine.name = machine.name();
        rosMachine.type = machine.type();
        rosMachine.state = machine.state();
        rosMachine.team_color = machine.team_color();
        rosMachine.pose.x = machine.pose().x();
        rosMachine.pose.y = machine.pose().y();
        rosMachine.pose.theta = machine.pose().ori();
        rosMachine.zone = uint8_t(machine.zone());
        //TODO loadedWith et lights si necessaire

        rosMachineInfo.machines.push_back(rosMachine);
    }

    return rosMachineInfo;
}

comm_msg::OrderInfo llsf2ros_orderInfo(const llsf_msgs::OrderInfo &llsfOrderInfo)
{
    comm_msg::OrderInfo rosOrderInfo;

    for(int i=0; i<llsfOrderInfo.orders_size(); ++i)
    {
        const llsf_msgs::Order &llsfOrder = llsfOrderInfo.orders(i);
        comm_msg::Order rosOrder;
        rosOrder.id = llsfOrder.id();
        rosOrder.complexity = llsfOrder.complexity();
        rosOrder.base_color = llsfOrder.base_color();
        for (int j=0; j < llsfOrder.ring_colors_size(); ++j)
        {
            rosOrder.ring_colors.push_back(uint8_t(llsfOrder.ring_colors(j)));
        }
        rosOrder.cap_color = llsfOrder.cap_color();
        rosOrder.quantity_requested = llsfOrder.quantity_requested();
        rosOrder.quantity_delivered_cyan = llsfOrder.quantity_delivered_cyan();
        rosOrder.quantity_delivered_magenta = llsfOrder.quantity_delivered_magenta();

        rosOrder.delivery_period_begin = llsfOrder.delivery_period_begin();
        rosOrder.delivery_period_end = llsfOrder.delivery_period_end();

        rosOrder.delivery_gate = llsfOrder.delivery_gate();

        rosOrderInfo.orders.push_back(rosOrder);
    }
    return rosOrderInfo;

}
