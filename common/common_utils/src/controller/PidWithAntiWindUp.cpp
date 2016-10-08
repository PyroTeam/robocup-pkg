/**
 * \file         PidWithAntiWindUp.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-18
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include "common_utils/controller/PidWithAntiWindUp.h"

namespace common_utils {

PidWithAntiWindUp::PidWithAntiWindUp(float Kp, float Ki, float Kd, float T, float lowLim, float highLim, float antiWindUpParam = 0.1):
    Pid(Kp, Ki, Kd, T), m_antiWindUpParam(antiWindUpParam)
{
    setLimits(lowLim, highLim);
}

PidWithAntiWindUp::~PidWithAntiWindUp()
{

}

void PidWithAntiWindUp::setLimits(float lowLim, float highLim)
{
    if (lowLim > highLim)
    {
        m_lowLim = highLim;
        m_highLim = lowLim;
    }
    else
    {
        m_lowLim = lowLim;
        m_highLim = highLim;
    }
}

void PidWithAntiWindUp::setAntiWindUpParam(float param)
{
    //il est recommandé d'utiliser une valeur entre 0.05 et 0.25
    m_antiWindUpParam = param;
}

float PidWithAntiWindUp::update(float err)
{
    float pidOut = 0.0, pidOutTemp = 0.0;
    //Utilisation de la méthode "Soft Integrator Anti-Windup"
    //peut-être modifiée à l'avenir

    pidOutTemp = m_Kp * err + m_Ki * m_I + m_Kd * (err - m_err)/m_T;
    m_err = err;

    if (pidOutTemp >= m_highLim)
    {
        pidOut = m_highLim;
        m_I += err * m_T * m_antiWindUpParam;
    }
    else if (pidOutTemp <= m_lowLim)
    {
        pidOut = m_lowLim;
        m_I += err * m_T * m_antiWindUpParam;
    }
    else
    {
        m_I += err * m_T;
    }


    return pidOut;

}

void PidWithAntiWindUp::reset()
{
    Pid::reset();
}

} // namespace common_utils
