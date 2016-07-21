/**
* \file        CapStation.h
* \class       CapStation
* \brief       classe qui stocke les données de la capstation
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef CAP_STATION_H
#define CAP_STATION_H

#include "Machine.h"

using namespace manager_msg;

class CapStation : public Machine
{
private:
    int m_blackCap;
    int m_greyCap;
    int m_stockID[3];
    int m_capID[3];

public:
    CapStation(int teamColor);
    CapStation(int teamColor, int number);
    virtual ~CapStation();

    int getGreyCap();
    int getBlackCap();
    int getStockage(int i);

    void majStockID(int i, int val);
    void majBlack(int nbNoir);
    void majGrey(int nbGris);

    void put_cap(int capColor);
    void take_cap();

    void uncap();
    void stock(int place);
    void destock(int place);
};

#endif
