/**
 * \file         PidWithAntiWindUp.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-18
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef COMMON_UTILS_PIDWITHANTIWINDUP_H_
#define COMMON_UTILS_PIDWITHANTIWINDUP_H_

#include "Pid.h"

namespace common_utils {

class PidWithAntiWindUp : public Pid
{
public:
    PidWithAntiWindUp(float Kp, float Ki, float Kd, float T);

    virtual ~PidWithAntiWindUp();

    virtual float update(float err) override;
protected:
};

} // namespace common_utils

#endif /* COMMON_UTILS_PIDWITHANTIWINDUP_H_ */
