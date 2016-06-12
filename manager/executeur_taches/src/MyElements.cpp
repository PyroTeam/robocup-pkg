#include "MyElements.h"

/* Constructeur */
MyElements::MyElements(int teamColor)
: m_bs(teamColor)
, m_rs1(1, teamColor)
, m_rs2(2, teamColor)
, m_cs1(1, teamColor)
, m_cs2(2, teamColor)
, m_ds(teamColor)
{}

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

Machine *MyElements::getMachineFromTag(int arTag, int teamColor)
{
	if (teamColor == CYAN)
	{
		switch(arTag)
		{
			case  C_CS1_IN    :
			case  C_CS1_OUT   :
				return &getCS1();

			case  C_CS2_IN    :
			case  C_CS2_OUT   :
				return &getCS2();

			case  C_RS1_IN    :
			case  C_RS1_OUT   :
				return &getRS1();

			case  C_RS2_IN    :
			case  C_RS2_OUT   :
				return &getRS2();

			case  C_BS_IN     :
			case  C_BS_OUT    :
				return &getBS();

			case  C_DS_IN     :
			case  C_DS_OUT    :
				return &getDS();

			default:
				break;
		}
	}

	if (teamColor == MAGENTA)
	{
		switch(arTag)
		{
			case  M_CS1_IN    :
			case  M_CS1_OUT   :
				return &getCS1();

			case  M_CS2_IN    :
			case  M_CS2_OUT   :
				return &getCS2();

			case  M_RS1_IN    :
			case  M_RS1_OUT   :
				return &getRS1();

			case  M_RS2_IN    :
			case  M_RS2_OUT   :
				return &getRS2();

			case  M_BS_IN     :
			case  M_BS_OUT    :
				return &getBS();

			case  M_DS_IN     :
			case  M_DS_OUT    :
				return &getDS();

			default:
				break;
		}
	}

	ROS_ERROR("Invalid ArTagID for this team/color: %d (teamcolor: %d)", arTag, teamColor);
	return nullptr;
}
