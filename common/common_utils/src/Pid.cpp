/**
 * \file
 * \brief      Un simple PID, directement tiré du path_tracker
 * \author     Valentin Vergez (valentin.vergez@gmail.com)
 * \date       Create : 2016-05-22
 * \date       Last modified : 2016-05-22
 * \copyright  2016, Association de Robotique de Polytech Lille All rights reserved
 */

 #include "common_utils/Pid.h"

Pid::Pid(float Kp, float Ki, float Kd, float T):
m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_err(0), m_I(0), m_T(T)
{
}

Pid::~Pid()
{
}

float Pid::update(float err)
{
	float cmd;

	m_I += err * m_T;
	cmd = m_Kp * err + m_Ki * m_I + m_Kd * (err - m_err)/m_T;
	m_err = err;

	return cmd;
};