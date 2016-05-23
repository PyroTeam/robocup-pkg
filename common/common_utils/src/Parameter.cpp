/**
 * \file
 *
 * \see			common_utils/Parameter.h
 *
 * \author      Valentin Vergez (valentin.vergez@gmail.com)
 * \date        2016-02-20
 * \copyright   2016, Association de Robotique de Polytech Lille
 * \license		BSD
 * \version
 */

#include "common_utils/Parameter.h"

Parameter::Parameter(ros::NodeHandle &nh, std::string name, float defaultValue)
: m_nh(nh)
, m_defaultValue(defaultValue)
, m_useDefault(false)
{
	m_fullName = m_nh.resolveName(name);
	ROS_DEBUG("Construct %s parameter object", m_fullName.c_str());

	if (!m_nh.hasParam(m_fullName))
	{
		ROS_WARN("Unable to find %s parameter. Will use default value : %f"
			, m_fullName.c_str(), m_defaultValue);
		m_useDefault = true;
	}
}

Parameter::~Parameter(void)
{
	ROS_DEBUG("Destroy %s parameter object", m_fullName.c_str());
}

float Parameter::get(void)
{
	double tmp_value;
	if (!m_useDefault)
	{
		if (!m_nh.getParamCached(m_fullName, tmp_value))
		{
			ROS_WARN("Unable to get %s parameter. Fallback on default value : %f"
				, m_fullName.c_str(), m_defaultValue);
			m_useDefault = true;
			return m_defaultValue;
		}

		return (float)tmp_value;
	}

	ROS_DEBUG("Parameter %s use default value : %f", m_fullName.c_str(), m_defaultValue);

	return m_defaultValue;
}

float Parameter::operator()(void)
{
	return this->get();
}