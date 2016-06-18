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

struct StIdle;
struct StRun;
struct StPause;
struct StActive;


struct PTmachine : sc::state_machine<PTmachine, StActive>
{
public:
    PTmachine():m_behavior(nullptr)
    {

    }

    void setBehavior(std::shared_ptr<MoveBehavior> &behavior)
    {
        m_behavior = behavior;
    }

    std::shared_ptr<MoveBehavior> & getBehavior()
    {
        return m_behavior;
    }
private:
    std::shared_ptr<MoveBehavior> m_behavior;

};


struct StActive : sc::state<StActive, PTmachine, StIdle>
{
public:
    typedef sc::transition< EvReset, StActive > reactions;

    StActive(my_context ctx) : sc::state<StActive, PTmachine, StIdle>( ctx )
    {
        ROS_INFO("PathTracking: Entering Active State");
        m_behavior = outermost_context().getBehavior();
    }

    std::shared_ptr<MoveBehavior> & getBehavior()
    {
        return m_behavior;
    }
private:
    std::shared_ptr<MoveBehavior> m_behavior;
};



struct StIdle : sc::simple_state<StIdle, StActive>
{
    typedef sc::transition<EvStart, StRun> reactions;

    StIdle()
    {
        ROS_INFO("PathTracking: Entering Idle State");
    }
    ~StIdle()
    {
        ROS_INFO("PathTracking: Leaving Idle State");
    }

};

struct StRun : sc::state<StRun, StActive>
{
    typedef mpl::list<
        sc::transition<EvStart, StRun>,
        sc::transition<EvStop, StIdle>,
        sc::transition<EvEndPath, StIdle>,
        sc::transition<EvPause, StPause>
        > reactions;

    StRun(my_context ctx) : sc::state<StRun, StActive>( ctx )
    {
        ROS_INFO("PathTracking: Entering Run State");
        std::shared_ptr<MoveBehavior> behav = outermost_context().getBehavior();
        geometry_msgs::Twist twist = behav->generateNewSetPoint();
    }

    ~StRun()
    {
        ROS_INFO("PathTracking: Leaving Run State");
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
        ROS_INFO("PathTracking: Entering Pause State");
    }
    ~StPause()
    {
        ROS_INFO("PathTracking: Leaving Pause State");
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
