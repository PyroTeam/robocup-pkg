/**
 * \file         PathTracking.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-12
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include "path_tracker/PathTracking.h"

PathTracking::PathTracking(std::string name, const std::shared_ptr<MoveBehavior> &moveBehavior):
    m_nh(),
    m_trackPath_action(m_nh, name, boost::bind(&PathTracking::executeCB, this, _1), false)
{

    m_behavior = moveBehavior;
    m_ptMachine.initiate();
    m_ptMachine.process_event(EvStart());
    m_trackPath_action.start();

}


PathTracking::~PathTracking()
{

}

void PathTracking::executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal)
{

}
