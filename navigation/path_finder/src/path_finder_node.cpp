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
#include "graph/GridMapGraph.h"
#include "graph/UpdateGridMapGraph.h"
#include "graph/Heuristic.h"
#include "search_algo/AStarSearch.h"
#include "search_algo/PointState.h"

int main(int argc, char **argv)
{
    int loopFreq = 50;

    ros::init(argc, argv, "path_finder");
    ros::NodeHandle nh;

    //test instantiation
    std::shared_ptr<Graph> graph(new GridMapGraph());

    std::shared_ptr<SearchAlgo> searchAlgo(new AStarSearch(graph));
    graph->setSearchAlgo(searchAlgo);
    std::shared_ptr<Heuristic> heuristic(new EuclidianHeuristic());
    graph->setHeuristic(heuristic);

    std::shared_ptr<UpdateGraph> updateGraph(new UpdateGridMapGraph("/map", graph));

    std::shared_ptr<State> startState(new PointState());
    std::shared_ptr<State> endState(new PointState());

    std::shared_ptr<PointState> pStart = std::dynamic_pointer_cast<PointState>(startState);
    pStart->set(0, 0);
    std::shared_ptr<PointState> pEnd = std::dynamic_pointer_cast<PointState>(endState);
    pEnd->set(3, 3);
    graph->search(startState, endState);

    ROS_INFO_STREAM("Path finder : launched...");

    ros::Rate loop_rate(loopFreq);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
