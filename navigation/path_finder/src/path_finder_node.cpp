/**
 * \file 		path_finder_node.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <iostream>
#include <memory>

#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include "graph/GridMapGraph.h"
#include "graph/UpdateGridMapGraph.h"
#include "graph/Heuristic.h"
#include "search_algo/AStarSearch.h"
#include "search_algo/PointState.h"
#include "search_algo/Path.h"
#include "path_finder/GeneratePath.h"


std::shared_ptr<Graph> graph(new GridMapGraph());


nav_msgs::Path g_pathFound;
nav_msgs::Path g_pathSmoothed;
double g_weightData = 0.45;
double g_weightSmooth = 0.35;

bool generatePath_callback( path_finder::GeneratePath::Request  &req,
                            path_finder::GeneratePath::Response &res);


int main(int argc, char **argv)
{
    int loopFreq = 10;

    ros::init(argc, argv, "path_finder");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Path finder : launched...");

    //paramêtres
    bool tieBreaking = false;
    nh.param<bool>("AstarTieBreaking", tieBreaking, false);
    nh.param<double>("weightData", g_weightData, 0.45);
    nh.param<double>("weightSmooth", g_weightSmooth, 0.35);


    //test instantiation
    std::shared_ptr<AStarSearch> searchAlgo(new AStarSearch(graph, true));
    searchAlgo->setTieBreaking(tieBreaking);
    graph->setSearchAlgo(searchAlgo);
    std::shared_ptr<Heuristic> heuristic(new EuclidianHeuristic());
    std::shared_ptr<Heuristic> heuristicDiag(new DiagonalHeuristic());
    graph->setHeuristic(heuristicDiag);
    std::shared_ptr<UpdateGraph> updateGraph(new UpdateGridMapGraph("/map", graph));

    ros::ServiceServer service = nh.advertiseService("generatePath", generatePath_callback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1000);
    ros::Publisher pathSmooth_pub = nh.advertise<nav_msgs::Path>("/pathSmooth", 1000);
    g_pathFound.header.stamp = ros::Time::now();
    g_pathFound.header.frame_id = "map";
    g_pathSmoothed.header.stamp = ros::Time::now();
    g_pathSmoothed.header.frame_id = "map";

    ros::Rate loop_rate(loopFreq);
    while (ros::ok())
    {
        path_pub.publish(g_pathFound);
        pathSmooth_pub.publish(g_pathSmoothed);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


bool generatePath_callback( path_finder::GeneratePath::Request  &req,
                            path_finder::GeneratePath::Response &res)
{
    ROS_INFO("GeneratePath request - ID : %d", req.id);

    res.requeteAcceptee = true;
    std::shared_ptr<State> startState(new PointState());
    std::shared_ptr<State> endState(new PointState());


    std::shared_ptr<PointState> pStart = std::dynamic_pointer_cast<PointState>(startState);
    pStart->set(req.Depart.position.x, req.Depart.position.y);
    //pStart->set(0, 0);
    std::shared_ptr<PointState> pEnd = std::dynamic_pointer_cast<PointState>(endState);
    pEnd->set(req.Arrivee.position.x, req.Arrivee.position.y);
    //pEnd->set(-2, 3);

    Path path, pathS;
    graph->search(startState, endState, path);
    if (path.empty())
    {
        ROS_INFO("Chemin vide");
    }
    else
    {

        ROS_INFO_STREAM("Taille chemin = " << path.size());
        g_pathFound.header.stamp = ros::Time::now();
        g_pathFound.header.frame_id = "map";
        g_pathFound.poses.clear();
        g_pathFound.poses = path.getPoses();

        pathS = path;
        pathS.setSmoothParam(g_weightData, g_weightSmooth);
        pathS.smooth();
        g_pathSmoothed.header.stamp = ros::Time::now();
        g_pathSmoothed.header.frame_id = "map";
        g_pathSmoothed.poses.clear();
        g_pathSmoothed.poses = pathS.getPoses();
    }

  return true;
}
