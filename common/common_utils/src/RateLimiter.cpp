/**
 * \file         RateLimiter.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-23
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include "common_utils/RateLimiter.h"

namespace common_utils {


RateLimiter::RateLimiter(float low, float up, float T)
{
    void setLimits(float low, float up);
    m_lastSignal = 0.0;
}


float RateLimiter::saturation(float signal, float min, float max)
{
    if (signal < min)
    {
        return min;
    }
    else if (signal > max)
    {
        return max;
    }
    else
    {
        return signal;
    }
}


void RateLimiter::setLimits(float low, float up)
{
    if (low < up)
    {
        m_upLimit = up;
        m_lowLimit = low;
    }
    else
    {
        m_upLimit = low;
        m_lowLimit = up;
    }
}

float RateLimiter::update(float signal)
{
    float sigtemp = signal - m_lastSignal;
    sigtemp = saturation(sigtemp, m_lowLimit*m_T, m_upLimit*m_T);
    sigtemp = sigtemp + m_lastSignal;
    m_lastSignal = sigtemp;
    return sigtemp;
}

void RateLimiter::reset()
{
    m_lastSignal = 0.0;
}

} // namespace common_utils
