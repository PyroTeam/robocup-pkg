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


#ifndef PATH_FINDER_POSESTATE_H_
#define PATH_FINDER_POSESTATE_H_

#include <geometry_msgs/Pose2D.h>
#include "State.h"

class PoseState : public State
{
public:
    PoseState() : State()
    {

    }
    virtual ~PoseState()
    {

    }
protected:
    geometry_msgs::Pose2D m_pose;
};

#endif /* PATH_FINDER_POINTSTATE_H_ */
