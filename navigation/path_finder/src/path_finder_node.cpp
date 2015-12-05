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
#include "PathPlanner.h"
#include "path_finder/GeneratePath.h"
#include "path_finder/AstarPath.h"


int main(int argc, char **argv)
{
    int loopFreq = 10;

    ros::init(argc, argv, "path_finder");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Path finder : launched...");

    //paramètres
    bool tieBreaking = false;
    nh.param<bool>("navigation/pathFinder/aStarTieBreaking", tieBreaking, false);

    //Création du graph et des algorithmes
    std::shared_ptr<Graph> graph(new GridMapGraph());
    std::shared_ptr<AStarSearch> searchAlgo(new AStarSearch(graph, true));
    searchAlgo->setTieBreaking(tieBreaking);
    graph->setSearchAlgo(searchAlgo);
    std::shared_ptr<Heuristic> heuristic(new EuclidianHeuristic());
    std::shared_ptr<Heuristic> heuristicDiag(new DiagonalHeuristic());
    graph->setHeuristic(heuristicDiag);
    std::shared_ptr<UpdateGraph> updateGraph(new UpdateGridMapGraph("/map", graph));


    //publication des path et pathSmooth pour le debug
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1000);
    ros::Publisher pathSmooth_pub = nh.advertise<nav_msgs::Path>("/pathSmooth", 1000);
    //AstarPath pour la compatibilité
    ros::Publisher aStarPath_pub = nh.advertise<path_finder::AstarPath>("/pathFound", 1000);

    PathPlanner pathPlanner(graph, "navigation/generatePath");

    ros::Rate loop_rate(loopFreq);
    while (ros::ok())
    {
        const nav_msgs::Path &p = pathPlanner.getPath();
        const nav_msgs::Path &ps = pathPlanner.getPath(true);
        path_pub.publish(p);
        pathSmooth_pub.publish(ps);

        path_finder::AstarPath asp;
        asp.id = pathPlanner.getPathId();
        asp.path = ps;
        aStarPath_pub.publish(asp);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
