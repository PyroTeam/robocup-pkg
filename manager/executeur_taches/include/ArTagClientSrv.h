/**
* \file 		ArTagClientSrv.h
* \class		ArTagClientSrv
* \brief		classe client service ArTag
* \author		Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date		2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef ARTAG_CLIENT_SRV_H
#define ARTAG_CLIENT_SRV_H

#include <ros/ros.h>
#include <trait_im_msg/artag.h>

class ArTagClienSrv
{
public:
    ArTagClienSrv();
    virtual  ~ArTagClienSrv();

    int16_t askForId();

private:
    int16_t m_id;
};

#endif
