#include <string>

#include "CapStation.h"
#include "manager_msg/activity.h"

/* Constructeur */
CapStation::CapStation(int teamColor)
: Machine(teamColor)
{
    m_name += "CS";
    m_faType = FinalApproachingGoal::CS;
    m_type = "CapStation";
    m_blackCap = 0;
    m_greyCap = 0;
    m_stockID[0] = 1, m_stockID[1] = 1, m_stockID[2] = 1;
    m_capID[0] = 1, m_capID[1] = 1, m_capID[2] = 1;
}

CapStation::CapStation(int teamColor, int number)
: CapStation(teamColor)
{
    m_name += std::to_string(number);
    assert(number >= 1);
    assert(number <= 2);
    if (number == 1)
    {
        m_activityType = activity::CS1;
    }
    else
    {
        m_activityType = activity::CS2;
    }
}

CapStation::~CapStation(){}

int CapStation::getGreyCap()
{
    return m_greyCap;
}
int CapStation::getBlackCap()
{
    return m_blackCap;
}
int CapStation::getStockage(int i)
{
    return  m_stockID[i];
}

void CapStation::majStockID(int i, int val)
{
    m_stockID[i] = val;
}

void CapStation::majBlack(int nbNoir)
{
    m_blackCap = nbNoir;
}

void CapStation::majGrey(int nbGris)
{
    m_greyCap = nbGris;
}

void CapStation::put_cap(int capColor)
{
    ROS_INFO("Putting a Cap, color : %d", capColor);

    goTo(m_entryMachine);

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::IN,
                 FinalApproachingGoal::CONVEYOR);

    let();

    //TODO: demander à la refbox un cap de couleur "color" )

    //TODO: attendre fin de livraison

    //XXX: Retourner qqch pour dire que la tâche est réalisée
}

void CapStation::take_cap()
{
    goTo(m_exitMachine);

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::OUT,
                 FinalApproachingGoal::CONVEYOR);

    grip();

    //XXX: Retourner qqch pour dire que la tâche est réalisée
}

void CapStation::stock(int id)
{
    int8_t place;

    ROS_INFO("Stocking @ place : %d", id);

    goTo(m_entryMachine);

    if(id == 0)      place = FinalApproachingGoal::S1;
    else if(id == 1) place = FinalApproachingGoal::S2;
    else if(id == 2) place = FinalApproachingGoal::S3;

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::IN,
                 place);

    let();

    majStockID(id, 1);
}

void CapStation::destock(int id)
{
    int8_t place;

    ROS_INFO("Destocking @ place : %d", id);

    goTo(m_entryMachine);

    if(id == 0)      place = FinalApproachingGoal::S1;
    else if(id == 1) place = FinalApproachingGoal::S2;
    else if(id == 2) place = FinalApproachingGoal::S3;

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::IN,
                 place);

    let();

    majStockID(id, 0);
}

void CapStation::uncap()
{
    int8_t place;

    ROS_INFO("Uncaping");

    // vérifier si on a déjà uncap avant
    if(m_capID[0] == 1)
    {
        place = FinalApproachingGoal::S1;
        m_capID[0] == 0;
        m_stockID[0] = 0;
    }
    else if(m_capID[1] == 1)
    {
        place = FinalApproachingGoal::S2;
        m_capID[1] == 0;
        m_stockID[1] = 0;
    }
    else if(m_capID[2] == 1)
    {
        place = FinalApproachingGoal::S3;
        m_capID[2] == 0;
        m_stockID[2] = 0;
    }

    goTo(m_entryMachine);

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::IN,
                 place);

    grip();

    startFinalAp(FinalApproachingGoal::CS,
                 FinalApproachingGoal::IN,
                 FinalApproachingGoal::CONVEYOR);

    let();

    //TODO: demander à la refbox de uncap (je sais pas si il faut le demander)
    // Dire qq part qu'on vient de uncap (voir case orderRequest::BRING_BASE_RS dans GtServerSrv.cpp)

    //XXX: Retourner qqch pour dire que la tâche est réalisée
}
