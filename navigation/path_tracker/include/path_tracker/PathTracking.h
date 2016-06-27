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

#ifndef PATH_TRACKER_PATHTRACKING_H_
#define PATH_TRACKER_PATHTRACKING_H_

#include <memory>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "deplacement_msg/TrackPathAction.h"
#include "MoveBehavior.h"

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/mpl/list.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct EvStart : sc::event<EvStart> {};
struct EvStop : sc::event<EvStop> {};
struct EvPause : sc::event<EvPause> {};
struct EvEndPath : sc::event<EvEndPath> {};
struct EvReset : sc::event< EvReset > {};
struct EvTimer : sc::event< EvTimer > {};

struct StIdle;
struct StRun;
struct StPause;
struct StActive;


struct PTmachine : sc::state_machine<PTmachine, StActive>
{
public:
    PTmachine():m_behavior(nullptr)
    {
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
    }

    void setBehavior(std::shared_ptr<MoveBehavior> &behavior)
    {
        m_behavior = behavior;

    }

    std::shared_ptr<MoveBehavior> & getBehavior()
    {
        return m_behavior;
    }

    ros::Publisher &getCmdVelPub()
    {
        return m_cmdVel_pub;
    }
private:
    std::shared_ptr<MoveBehavior> m_behavior;
    ros::NodeHandle m_nh;
    ros::Publisher m_cmdVel_pub;
};


struct StActive : sc::state<StActive, PTmachine, StIdle>
{
public:
    typedef sc::transition< EvReset, StActive > reactions;

    StActive(my_context ctx) :
        sc::state<StActive, PTmachine, StIdle>( ctx ),
        m_behavior(nullptr)
    {
        ROS_DEBUG("PathTracking: Entering Active State");

        m_behavior = outermost_context().getBehavior();

    }

    std::shared_ptr<MoveBehavior> & getBehavior()
    {
        return m_behavior;
    }

    void updateCommand(const EvTimer &evTimer)
    {
        //générer la nouvelle commande
        geometry_msgs::Twist twist = m_behavior->generateNewSetpoint();

        //appliquer la commande
        outermost_context().getCmdVelPub().publish(twist);
    }

    void startNewPath(const EvStart &evStart)
    {
        if (m_behavior == nullptr)
        {
            ROS_ERROR("MoveBehavior not initialized");
            return;
        }
        m_behavior->startTraj();
    }


private:
    std::shared_ptr<MoveBehavior> m_behavior;
};



struct StIdle : sc::simple_state<StIdle, StActive>
{
    typedef sc::transition<EvStart, StRun, StActive, &StActive::startNewPath> reactions;

    StIdle()
    {
        ROS_DEBUG("PathTracking: Entering Idle State");
    }
    ~StIdle()
    {
        ROS_DEBUG("PathTracking: Leaving Idle State");
    }

};

struct StRun : sc::simple_state<StRun, StActive>
{
    typedef mpl::list<
        sc::transition<EvStart, StRun, StActive, &StActive::startNewPath>,
        sc::transition<EvStop, StIdle>,
        sc::transition<EvEndPath, StIdle>,
        sc::transition<EvPause, StPause>,
        sc::transition<EvTimer, StRun, StActive, &StActive::updateCommand>
        > reactions;

    StRun()
    {
        ROS_DEBUG("PathTracking: Entering Run State");
    }

    ~StRun()
    {
        ROS_DEBUG("PathTracking: Leaving Run State");
    }

};


struct StPause : sc::simple_state<StPause, StActive>
{
    typedef mpl::list<
        sc::transition<EvStart, StRun>,
        sc::transition<EvStop, StIdle>
        > reactions;

    StPause()
    {
        ROS_DEBUG("PathTracking: Entering Pause State");
    }
    ~StPause()
    {
        ROS_DEBUG("PathTracking: Leaving Pause State");
    }

};


class PathTracking
{
public:
    PathTracking(std::string name, const std::shared_ptr<MoveBehavior> &moveBehavior);
    virtual ~PathTracking();

protected:
    ros::NodeHandle m_nh;
    actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> m_trackPath_action;

    PTmachine m_ptMachine;
    std::shared_ptr<MoveBehavior> m_behavior;

    void executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal);
};

#endif /* PATH_TRACKER_PATHTRACKING_H_ */
