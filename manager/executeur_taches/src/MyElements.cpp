#include "MyElements.h"

/* Constructeur */
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

/* Destructeur */
MyElements::~MyElements(){}

/* Méthodes */
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

CapStation& MyElements::getCS1()
{
	return m_cs1;
}

CapStation& MyElements::getCS2()
{
	return m_cs2;
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

		case  C_CS2_IN    :
		case  C_CS2_OUT   :
    case  M_CS2_IN    :
    case  M_CS2_OUT   :
			return &getCS2();

		case  C_RS1_IN    :
		case  C_RS1_OUT   :
    case  M_RS1_IN    :
    case  M_RS1_OUT   :
			return &getRS1();

    case  C_RS2_IN    :
		case  C_RS2_OUT   :
    case  M_RS2_IN    :
    case  M_RS2_OUT   :
			return &getRS2();

    case  C_BS_IN     :
		case  C_BS_OUT    :
    case  M_BS_IN     :
    case  M_BS_OUT    :
			return &getBS();

		case  C_DS_IN     :
		case  C_DS_OUT    :
    case  M_DS_IN     :
    case  M_DS_OUT    :
			return &getDS();

		default:
			break;
	}

	ROS_ERROR("Invalid ArTagID : %d ", arTag);
	return nullptr;
}
