/**
* \file        MyElements.h
* \class       MyElements
* \brief       classe pour les différentes machines
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef MYELEMENTS_H
#define MYELEMENTS_H

#include "common_utils/types.h"
#include "manager_msg/order.h"

#include "Machine.h"
#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"

class MyElements
{
private:
    BaseStation m_bs;
    RingStation m_rs1;
    RingStation m_rs2;
    CapStation m_cs1;
    CapStation m_cs2;
    DeliveryStation m_ds;

    int m_color;

public:
    MyElements(int teamColor);
    virtual ~MyElements();

    BaseStation& getBS();
    RingStation& getRS1();
    RingStation& getRS2();
    CapStation&  getCS1();
    CapStation&  getCS2();
    DeliveryStation& getDS();

    RingStation& getRS(int ringColor);
    CapStation&  getCS(int capColor);

    Machine *getMachineFromTag(int arTag);

};

#endif
