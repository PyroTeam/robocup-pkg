/**
 * \file         Controller.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-18
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef COMMON_UTILS_CONTROLLER_H_
#define COMMON_UTILS_CONTROLLER_H_

namespace common_utils {

class Controller
{
public:
    Controller()
    {
    }

    virtual ~Controller()
    {

    }

    virtual float update(float err) = 0;
protected:
};

} // namespace common_utils

#endif /* COMMON_UTILS_CONTROLLER_H_ */
