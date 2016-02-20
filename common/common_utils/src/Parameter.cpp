/**
 * \file
 *
 * \see			common_utils/Parameter.h
 *
 * \author      Valentin Vergez (valentin.vergez@gmail.com)
 * \date        2016-02-20
 * \copyright   PyroTeam, Polytech-Lille
 * \license		LGPLv3
 * \version
 */

#include "Parameter.h"

Parameter::Parameter(ros::NodeHandle nh, std::string fullName)
: m_nh(nh)
, m_fullName(fullName)
, m_mode(FORCE_REFRESH)
{
	// TODO
}

Parameter::~Parameter(void)
{
	// TODO
}

float Parameter::get(void)
{
	// TODO

	return 0.0;
}

float Parameter::operator()(void)
{
	//TODO

	return this->get();
}

void Parameter::refresh(void)
{
	//TODO

}

void Parameter::setMode(parameterCacheMode_t mode)
{

}

parameterCacheMode_t Parameter::getMode(void)
{

}