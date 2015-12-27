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

#include <ros/ros.h>
#include "PathPlanner.h"
#include "graph/Graph.h"
#include "common_utils/Chrono.h"

PathPlanner::PathPlanner(const std::shared_ptr<Graph> &graph, std::string name) : m_graph(graph), m_id(0)
{
    m_pathFound.header.stamp = ros::Time::now();
    m_pathFound.header.frame_id = "map";
    m_pathSmoothed.header.stamp = ros::Time::now();
    m_pathSmoothed.header.frame_id = "map";
    m_path_srv = m_nh.advertiseService(name, &PathPlanner::generatePath_callback, this);

}

PathPlanner::~PathPlanner()
{

}

bool PathPlanner::generatePath_callback(path_finder::GeneratePath::Request  &req,
                                        path_finder::GeneratePath::Response &res)
{
    ROS_INFO("GeneratePath request - ID : %d", req.id);

    m_id = req.id;
    //paramètres
    double weightData = 0.45;
    double weightSmooth = 0.35;
    m_nh.param<double>("navigation/pathFinder/weightData", weightData, 0.45);
    m_nh.param<double>("navigation/pathFinder/weightSmooth", weightSmooth, 0.35);

    res.requeteAcceptee = true;
    std::shared_ptr<State> startState(new PointState());
    std::shared_ptr<State> endState(new PointState());


    std::shared_ptr<PointState> pStart = std::dynamic_pointer_cast<PointState>(startState);
    pStart->set(req.Depart.position.x, req.Depart.position.y);
    std::shared_ptr<PointState> pEnd = std::dynamic_pointer_cast<PointState>(endState);
    pEnd->set(req.Arrivee.position.x, req.Arrivee.position.y);

    Path path, pathS;

    common_utils::HighResChrono chrono;
    chrono.start();
    m_graph->search(startState, endState, path);
    chrono.stop();
    ROS_INFO_STREAM("Time to generate the path : " << chrono);

    if (path.empty())
    {
        ROS_INFO("Chemin vide");
    }
    else
    {
        ROS_INFO_STREAM("Taille chemin = " << path.size());
        m_pathFound.header.stamp = ros::Time::now();
        m_pathFound.header.frame_id = "map";
        m_pathFound.poses.clear();
        m_pathFound.poses = path.getPoses();

        pathS = path;
        pathS.setSmoothParam(weightData, weightSmooth);
        chrono.start();
        pathS.smooth();
        chrono.stop();
        ROS_INFO_STREAM("Time to smooth the path : " << chrono);
        m_pathSmoothed.header.stamp = ros::Time::now();
        m_pathSmoothed.header.frame_id = "map";
        m_pathSmoothed.poses.clear();
        m_pathSmoothed.poses = pathS.getPoses();

        //orientation finale
        m_pathFound.poses.back().pose.orientation = req.Arrivee.orientation;
        m_pathSmoothed.poses.back().pose.orientation = req.Arrivee.orientation;
    }

    return true;
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

int PathPlanner::getPathId() const
{
    return m_id;
}
