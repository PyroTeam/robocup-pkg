#include <string>
#include "BaseStation.h"
#include <common_utils/types.h>

using namespace common_utils;

BaseStation::BaseStation(int teamColor)
: Machine(teamColor)
{
    m_name += "BS";
    m_faType = FinalApproachingGoal::BS;
    m_activityType = activity::BS;
    m_type = "BaseStation";
}

BaseStation::~BaseStation(){}

bool BaseStation::take(int color)
{
    goTo(m_exitMachine);

    //TODO: demander à la refbox une base de couleur "couleur" )

    //TODO: attendre fin de livraison

    // Accoster machine
    startFinalAp(FinalApproachingGoal::BS,
                 FinalApproachingGoal::OUT,
                 FinalApproachingGoal::CONVEYOR);

    // Prendre base
    grip();
}
