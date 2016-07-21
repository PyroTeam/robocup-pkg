/**
* \file        RingStation.h
* \class       RingStation
* \brief       classe qui stocke les données de la ringstation
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef RINGSTATION_H
#define RINGSTATION_H

#include "Machine.h"

using namespace manager_msg;

class RingStation : public Machine
{
private:
    int m_greenRing;
    int m_yellowRing;
    int m_blueRing;
    int m_orangeRing;

public:
    RingStation(int teamColor);
    RingStation(int teamColor, int number);

    /* Destructeur */
    virtual ~RingStation();

    /* Méthodes */
    int getGreenRing();
    int getYellowRing();
    int getBlueRing();
    int getOrangeRing();

    void majGreen(int nbVert);
    void majYellow(int nbJaune);
    void majBlue(int nbBleu);
    void majOrange(int nbOrange);

    void put_ring(int ringColor);
    void take_ring();
    void bring_base();
};

#endif
