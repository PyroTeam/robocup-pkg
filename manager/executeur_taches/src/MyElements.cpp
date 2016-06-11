#include "MyElements.h"

/* Constructeur */
MyElements::MyElements()
: m_bs()
, m_rs1(1)
, m_rs2(2)
, m_cs1(1)
, m_cs2(2)
, m_ds()
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
