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


class MPS
{
public:
    MPS();
    ~MPS();

    geometry_msgs::Pose2D pose;
    int zone;
    int idIn;
    int idOut;
    bool isHere;
    bool orientationOk;
};

#endif
