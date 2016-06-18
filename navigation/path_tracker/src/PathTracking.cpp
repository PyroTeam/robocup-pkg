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
    //m_ptMachine.process_event(EvStart());
    m_trackPath_action.start();

}


PathTracking::~PathTracking()
{

}

void PathTracking::executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal)
{
    deplacement_msg::TrackPathFeedback feedback;
    deplacement_msg::TrackPathResult result;

    //process goal command
    switch (goal->command)
    {
    case deplacement_msg::TrackPathGoal::CMD_START:
        m_ptMachine.process_event(EvStart());
        break;
    case deplacement_msg::TrackPathGoal::CMD_STOP:
        m_ptMachine.process_event(EvStop());
        break;
    case deplacement_msg::TrackPathGoal::CMD_PAUSE:
        m_ptMachine.process_event(EvPause());
        break;
    default:
        break;
    }

    result.status = deplacement_msg::TrackPathResult::STATUS_FINISHED;
    result.error = deplacement_msg::TrackPathResult::ERR_NONE;
    m_trackPath_action.setSucceeded(result);

}
