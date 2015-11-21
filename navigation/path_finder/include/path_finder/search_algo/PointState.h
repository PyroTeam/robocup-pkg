/**
 * \file 		PointState.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#ifndef PATH_FINDER_POINTSTATE_H_
#define PATH_FINDER_POINTSTATE_H_

#include <geometry_msgs/Point.h>
#include "State.h"

class PointState : public State
{
public:
    PointState() : State()
    {

    }
    virtual ~PointState()
    {
        
    }
protected:
    geometry_msgs::Point m_point;
};

#endif /* PATH_FINDER_POINTSTATE_H_ */
