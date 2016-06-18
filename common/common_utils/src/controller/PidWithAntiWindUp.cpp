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

PidWithAntiWindUp::PidWithAntiWindUp(float Kp, float Ki, float Kd, float T):
    Pid(Kp, Ki, Kd, T)
{

}

PidWithAntiWindUp::~PidWithAntiWindUp()
{

}

float PidWithAntiWindUp::update(float err)
{
    //TODO: Implémenter la version avec anti windup
    Pid::update(err);
}

} // namespace common_utils
