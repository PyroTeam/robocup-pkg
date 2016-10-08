/**
 * \file         entryPoint.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 *               Tissot Elise
 * \date         2015-04-11
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "entryPoint.h"

EntryPoint::EntryPoint(std::shared_ptr<UdpPeer> &udpPeer, const std::string &name):m_udpPeer(udpPeer), m_name(name)
{

}

EntryPoint::~EntryPoint()
{

}
