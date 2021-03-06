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
#include "path_tracker/SwitchModeBehavior.h"

PathTracking::PathTracking(std::string name, const std::shared_ptr<MoveBehavior> &moveBehavior):
    m_nh(),
    m_trackPath_action(m_nh, name, boost::bind(&PathTracking::executeCB, this, _1), false)
{

    m_behavior = moveBehavior;
    m_ptMachine.setBehavior(m_behavior);
    m_ptMachine.initiate();

    m_trackPath_action.start();
}


PathTracking::~PathTracking()
{

}

void PathTracking::executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal)
{
    deplacement_msg::TrackPathFeedback feedback;
    deplacement_msg::TrackPathResult result;

    ros::Time trackBegin = ros::Time::now();

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


    bool isTrajEnd = false;
    if (m_ptMachine.state_downcast<const StRun *>() != 0)
    {

        ros::Rate loopRate(10);//TODO à parametrer

        while(ros::ok()
            && m_ptMachine.state_downcast<const StRun *>() != 0
            && !m_trackPath_action.isPreemptRequested()
            && !m_behavior->isTrajectoryEnd()
            && !m_behavior->isObstacleUnavoidable())
        {

            m_ptMachine.process_event(EvTimer());
            feedback.percentComplete = m_behavior->getPercentComplete()*100.0;
            feedback.processingTime = ros::Time::now() - trackBegin;
            // TODO duration avoidanceTime
            feedback.trackPathError = m_behavior->getPathError();

            //TODO à nettoyer
            if (std::shared_ptr<SwitchModeBehavior> b = std::dynamic_pointer_cast<SwitchModeBehavior>(m_behavior))
            {

                BehaviorMode_t m = b->getMode();
                switch (m)
                {
                default:
                case BehaviorMode_t::FOLLOW:
                    feedback.mode = deplacement_msg::TrackPathFeedback::MODE_TRACKPATH;
                    break;
                case BehaviorMode_t::AVOID:
                    feedback.mode = deplacement_msg::TrackPathFeedback::MODE_AVOID;
                    break;
                }
            }

            m_trackPath_action.publishFeedback(feedback);

            ros::spinOnce();
            loopRate.sleep();
        }


        //fin d'action, retourner le résultat
        if (m_behavior->isTrajectoryEnd())
        {
            m_ptMachine.process_event(EvEndPath());

            result.status = deplacement_msg::TrackPathResult::STATUS_FINISHED;
            result.error = deplacement_msg::TrackPathResult::ERR_NONE;
            m_trackPath_action.setSucceeded(result);
        }
        else if (m_behavior->isObstacleUnavoidable())
        {
            m_ptMachine.process_event(EvEndPath());
            result.status = deplacement_msg::TrackPathResult::STATUS_INTERRUPTED;
            result.error = deplacement_msg::TrackPathResult::ERR_AVOIDANCE_UNAVOIDABLE;
            m_trackPath_action.setAborted(result);
        }
        else
        {
            result.status = deplacement_msg::TrackPathResult::STATUS_FINISHED;
            result.error = deplacement_msg::TrackPathResult::ERR_UNKNOWN;
            m_trackPath_action.setAborted(result);
        }
    }
    else
    {
        result.status = deplacement_msg::TrackPathResult::STATUS_FINISHED;
        result.error = deplacement_msg::TrackPathResult::ERR_NONE;
        m_trackPath_action.setSucceeded(result);
    }
}
