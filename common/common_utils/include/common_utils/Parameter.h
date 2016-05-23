/**
 * \file
 *
 * \brief		Parameter class
 *
 * \author      Valentin Vergez (valentin.vergez@gmail.com)
 * \date        2016-02-20
 * \copyright   PyroTeam, Polytech-Lille
 * \license		LGPLv3
 * \version
 */

#ifndef _COMMON_UTILS__PARAMETER__H_
#define _COMMON_UTILS__PARAMETER__H_

#include <string>

#include <ros/ros.h>

class Parameter
{
public:
	Parameter(ros::NodeHandle &nh, std::string name, float defaultValue);
	~Parameter(void);
	float get(void);
	float operator()(void);

private:
	ros::NodeHandle m_nh;
	std::string m_fullName;
	float m_defaultValue;
	bool m_useDefault;
};

#endif //_COMMON_UTILS__PARAMETER__H_