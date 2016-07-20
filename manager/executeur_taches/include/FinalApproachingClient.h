/**
* \file        FinalApproachingClient.h
* \class       FinalApproachingClient
* \brief       classe client pour l'approche finale
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef FINAL_APPROACHING_CLIENT_H
#define FINAL_APPROACHING_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <cstdint>

#include <common_utils/Parameter.h>
#include <final_approach_msg/FinalApproachingAction.h>


class FinalApproachingClient
{
public:
  /* Constructeur */
  FinalApproachingClient();

  /* Destructeur */
  virtual  ~FinalApproachingClient();

  /* Méthodes */
  void starting(int8_t machineType, int8_t machineSide, int8_t machineParameter);
  bool getSuccess();
private:
  ros::NodeHandle m_nh;
  bool m_success;
  Parameter m_globalTimeout;
  Parameter m_lightAsservTimeout;
};
#endif
