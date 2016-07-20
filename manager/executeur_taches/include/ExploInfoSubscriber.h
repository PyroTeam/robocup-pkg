/**
 * \file        ExploInfoSubscriber.h
 * \class       ExploInfoSubscriber
 * \brief       classe pour interpréter des données pour la phase d'exploration
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef EXPLO_INFO_SUBSCRIBER_H
#define EXPLO_INFO_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>

#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ExplorationSignal.h"
#include "comm_msg/ExplorationZone.h"
#include "comm_msg/LightSpec.h"

#include "FeuClientAction.h"

class ExploInfoSubscriber
{
  public:
    /* Constructeur */
    ExploInfoSubscriber();

    /* Destructeur */
    virtual  ~ExploInfoSubscriber();

    /* Méthodes */
    void tesCallback(const comm_msg::ExplorationInfo &msg);
    void interpretationFeu();

    std::string type;
    uint8_t zone;

    std::vector<comm_msg::LightSpec> m_lSpec;
    std::vector<comm_msg::ExplorationSignal>  m_signals;
    std::vector<comm_msg::ExplorationZone>  m_zones;

    ros::Subscriber m_sub;
};
#endif
