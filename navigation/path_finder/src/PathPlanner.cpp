/**
 * \file 		PathPlanner.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-28
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <future>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "PathPlanner.h"
#include "graph/Graph.h"
#include "common_utils/Chrono.h"

PathPlanner::PathPlanner(const std::shared_ptr<Graph> &graph, std::string name) :
    m_graph(graph),
    m_path_as(m_nh, name, boost::bind(&PathPlanner::generatePathExecute_callback, this, _1), false)
{
    std::string tf_prefix;
    m_nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    m_pathFound.header.stamp = ros::Time::now();
    m_pathFound.header.frame_id = tf_prefix+"map";
    m_pathSmoothed.header.stamp = ros::Time::now();
    m_pathSmoothed.header.frame_id = tf_prefix+"map";

    m_path_as.start();

}

PathPlanner::~PathPlanner()
{

}

void PathPlanner::generatePathExecute_callback(const deplacement_msg::GeneratePathGoalConstPtr &goal)
{
    bool success = true;

    //paramètres
    double weightData = 0.45;
    double weightSmooth = 0.35;
    m_nh.param<double>("navigation/pathFinder/weightData", weightData, 0.45);
    m_nh.param<double>("navigation/pathFinder/weightSmooth", weightSmooth, 0.35);
    std::string tf_prefix;
    m_nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    std::shared_ptr<State> startState(new PointState());
    std::shared_ptr<State> endState(new PointState());
    std::shared_ptr<PointState> pStart = std::dynamic_pointer_cast<PointState>(startState);
    pStart->set(goal->start.x, goal->start.y);
    std::shared_ptr<PointState> pEnd = std::dynamic_pointer_cast<PointState>(endState);
    pEnd->set(goal->goal.x, goal->goal.y);

    Path path, pathS;

    std::future<void> searchResult = std::async (std::launch::async, &Graph::search, m_graph, std::ref(startState), std::ref(endState), std::ref(path));
    std::chrono::milliseconds span (50);

    ros::Time beginTime = ros::Time::now();
    bool isTimeout = false;

    while (searchResult.wait_for(span)!=std::future_status::ready  && !isTimeout && ros::ok())
    {
        m_pathFeedback.processingTime = ros::Duration(ros::Time::now() - beginTime);
        m_path_as.publishFeedback(m_pathFeedback);

        if (m_path_as.isPreemptRequested() || !ros::ok())
        {
            m_graph->cancelSearch();
            ROS_INFO("PathPlanning Preempted");
            // set the action state to preempted
            m_path_as.setPreempted();
            success = false;
        }
        if (m_pathFeedback.processingTime > goal->timeout)
        {
            isTimeout = true;
            m_graph->cancelSearch();
            ROS_INFO("PathPlanning timeout");
        }
        ros::spinOnce();
    }
    searchResult.get();

    if (isTimeout)
    {
        m_pathResult.result = deplacement_msg::GeneratePathResult::ERROR_TIMEOUT;
    }
    else if (path.empty())
    {
        ROS_INFO("Chemin vide");
        m_pathResult.result = deplacement_msg::GeneratePathResult::ERROR_NO_PATH;
    }
    else
    {
        ROS_INFO_STREAM("Taille chemin = " << path.size());
        m_pathFound.header.stamp = ros::Time::now();
        m_pathFound.header.frame_id = tf_prefix+"map";
        m_pathFound.poses.clear();
        m_pathFound.poses = path.getPoses();

        pathS = path;
        pathS.setSmoothParam(weightData, weightSmooth);
        pathS.smooth();
        m_pathSmoothed.header.stamp = ros::Time::now();
        m_pathSmoothed.header.frame_id = tf_prefix+"map";
        m_pathSmoothed.poses.clear();
        m_pathSmoothed.poses = pathS.getPoses();

        //orientation finale
        m_pathFound.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(goal->goal.theta);
        m_pathSmoothed.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(goal->goal.theta);

        m_pathResult.result = deplacement_msg::GeneratePathResult::SUCCESS;
    }

    if(success)
    {
        ROS_INFO("Path finding : Succeeded");
        // set the action state to succeeded
        m_path_as.setSucceeded(m_pathResult);
    }
}

const nav_msgs::Path &PathPlanner::getPath(bool smoothed) const
{
    if(smoothed)
    {
        return m_pathSmoothed;
    }
    else
    {
        return m_pathFound;
    }
}
