#include "MyElements.h"

MyElements::MyElements(int teamColor)
: m_bs(teamColor)
, m_rs1(teamColor, 1)
, m_rs2(teamColor, 2)
, m_cs1(teamColor, 1)
, m_cs2(teamColor, 2)
, m_ds(teamColor)
{
    m_color = teamColor;
}

MyElements::~MyElements(){}

BaseStation& MyElements::getBS()
{
    return m_bs;
}

RingStation& MyElements::getRS1()
{
    return m_rs1;
}

RingStation& MyElements::getRS2()
{
    return m_rs2;
}

RingStation& MyElements::getRS(int ringColor)
{
    //TODO: setter les couleurs de RING (pas fait !)
    switch (ringColor)
    {
        case orderRequest::GREEN :
            return (m_rs1.getGreenRing() != 0) ? m_rs1 : m_rs2;
        break;
        case orderRequest::YELLOW :
            return (m_rs1.getYellowRing() != 0) ? m_rs1 : m_rs2;
        break;
        case orderRequest::BLUE :
            return (m_rs1.getBlueRing() != 0) ? m_rs1 : m_rs2;
        break;
        case orderRequest::ORANGE :
            return (m_rs1.getOrangeRing() != 0) ? m_rs1 : m_rs2;
        break;
        default :
        //TODO: TBD
        break;
    }
}

CapStation& MyElements::getCS1()
{
    return m_cs1;
}

CapStation& MyElements::getCS2()
{
    return m_cs2;
}


CapStation& MyElements::getCS(int capColor)
{
    //TODO: setter les couleurs de CAP (pas fait !)
    switch (capColor)
    {
        case orderRequest::GREY :
            return (m_cs1.getGreyCap() != 0) ? m_cs1 : m_cs2;
        break;
        case orderRequest::BLACK :
            return (m_cs1.getBlackCap() != 0) ? m_cs1 : m_cs2;
        break;
        default :
            //TODO: TBD
        break;
    }
}


DeliveryStation& MyElements::getDS()
{
    return m_ds;
}

Machine *MyElements::getMachineFromTag(int arTag)
{
    switch(arTag)
    {
        case  C_CS1_IN    :
        case  C_CS1_OUT   :
        case  M_CS1_IN    :
        case  M_CS1_OUT   :
            return &getCS1();
        break;

        case  C_CS2_IN    :
        case  C_CS2_OUT   :
        case  M_CS2_IN    :
        case  M_CS2_OUT   :
            return &getCS2();
        break;

        case  C_RS1_IN    :
        case  C_RS1_OUT   :
        case  M_RS1_IN    :
        case  M_RS1_OUT   :
            return &getRS1();
        break;

        case  C_RS2_IN    :
        case  C_RS2_OUT   :
        case  M_RS2_IN    :
        case  M_RS2_OUT   :
            return &getRS2();
        break;

        case  C_BS_IN     :
        case  C_BS_OUT    :
        case  M_BS_IN     :
        case  M_BS_OUT    :
            return &getBS();
        break;

        case  C_DS_IN     :
        case  C_DS_OUT    :
        case  M_DS_IN     :
        case  M_DS_OUT    :
            return &getDS();
        break;

        default:
            ROS_ERROR("Invalid ArTagID : %d ", arTag);
            return nullptr;
        break;
    }
}
