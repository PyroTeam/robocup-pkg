/**
 * \file         RateLimiter.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-23
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef COMMON_UTILS_RATELIMITER_H_
#define COMMON_UTILS_RATELIMITER_H_

namespace common_utils {

class RateLimiter
{
public:
    RateLimiter(float low, float up, float T);
    virtual ~RateLimiter()
    {

    }

    void setLimits(float low, float up);
    float update(float signal);
    void reset();

    static float saturation(float signal, float min, float max);
protected:
    float m_T;
    float m_upLimit;
    float m_lowLimit;
    float m_lastSignal;

};

} // namespace common_utils

#endif /* COMMON_UTILS_RATELIMITER_H_ */
