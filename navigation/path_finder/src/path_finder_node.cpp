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
#include "graph/gridMapGraph.h"
#include "graph/updateGridMapGraph.h"
#include "search_algo/aStarSearch.h"

int main(int argc, char **argv)
{
    int loopFreq = 50;

    ros::init(argc, argv, "path_finder");
    ros::NodeHandle nh;

    //test instantiation
    std::shared_ptr<Graph> graph(new GridMapGraph());
    std::shared_ptr<SearchAlgo> searchAlgo(new AStarSearch(graph));
    graph->search();

    ROS_INFO_STREAM("Path finder : launched...");

    ros::Rate loop_rate(loopFreq);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
