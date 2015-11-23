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
#include "path_finder/GeneratePath.h"

std::shared_ptr<Graph> graph(new GridMapGraph());


nav_msgs::Path pathFound;

bool generatePath_callback( path_finder::GeneratePath::Request  &req,
                            path_finder::GeneratePath::Response &res);


int main(int argc, char **argv)
{
    int loopFreq = 10;

    ros::init(argc, argv, "path_finder");

    ros::NodeHandle nh;

    ROS_INFO_STREAM("Path finder : launched...");

    //test instantiation
    std::shared_ptr<SearchAlgo> searchAlgo(new AStarSearch(graph));
    graph->setSearchAlgo(searchAlgo);
    std::shared_ptr<Heuristic> heuristic(new EuclidianHeuristic());
    std::shared_ptr<Heuristic> heuristicDiag(new DiagonalHeuristic());
    graph->setHeuristic(heuristicDiag);
    std::shared_ptr<UpdateGraph> updateGraph(new UpdateGridMapGraph("/map", graph));


    ros::ServiceServer service = nh.advertiseService("generatePath", generatePath_callback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1000);
    ros::Rate loop_rate(loopFreq);
    while (ros::ok())
    {
        path_pub.publish(pathFound);
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

    std::list<std::shared_ptr<State>> path;
    graph->search(startState, endState, path);
    if (path.empty())
    {
        ROS_INFO("Chemin vide");
    }
    else
    {
        ROS_INFO_STREAM("Taille chemin = " << path.size());
        pathFound.header.stamp = ros::Time::now();
        pathFound.header.frame_id = "map";
        pathFound.poses.clear();
        for(auto p : path)
        {
            std::shared_ptr<PointState> ps = std::dynamic_pointer_cast<PointState>(p);
            geometry_msgs::PoseStamped tempPose;
            tempPose.pose.position = ps->get();
            pathFound.poses.push_back(tempPose);
        }
    }

  return true;
}
