/**
* \file        MPS.h
* \class       MPS
* \brief       classe pour stocker la position et l'orientation d'une machine
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef MPS_H
#define MPS_H

#include "geometry_msgs/Pose2D.h"

namespace common_utils {

class MPS
{
private:
    geometry_msgs::Pose2D m_pose;
    int m_zone;
    int m_idIn;
    int m_idOut;
    bool m_orientationOk;

public:
    MPS();
    MPS(const geometry_msgs::Pose2D &pose);
    ~MPS();

    void pose(const geometry_msgs::Pose2D &pose);
    void zone(int area);
    void id(int ARTagId);
    void setOrientation();

    geometry_msgs::Pose2D pose(){return m_pose;}
    int zone(){return m_zone;}
    int idIn(){return m_idIn;}
    int idOut(){return m_idOut;}
    bool checkOrientation(){return m_orientationOk;}
    deplacement_msg::MPS msg();
};

} // namespace common_utils

#endif
