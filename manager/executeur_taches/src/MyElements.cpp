#include "MyElements.h"

/* Constructeur */
MyElements::MyElements(){}

/* Destructeur */
MyElements::~MyElements(){}

/* Méthodes */
BaseStation MyElements::getBS()
{
	return this->m_bs;
}
RingStation MyElements::getRS1()
{
	return this->m_rs1;
}
RingStation MyElements::getRS2()
{
	return this->m_rs2;
}
CapStation MyElements::getCS1()
{
	return this->m_cs1;
}
CapStation MyElements::getCS2()
{
	return this->m_cs2;
}
DeliveryStation MyElements::getDS()
{
	return this->m_ds;
}