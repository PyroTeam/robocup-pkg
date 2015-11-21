/**
 * \file 		AStarSearch.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <iostream>
#include "ros/ros.h"
#include "search_algo/AStarSearch.h"

AStarSearch::AStarSearch(const std::shared_ptr<Graph> &graph) : SearchAlgo(graph)
{

}

AStarSearch::~AStarSearch()
{

}

void AStarSearch::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState)
{
    ROS_INFO_STREAM("PahtFinder : Recherche un chemin de \n" << *startState << " à \n" << *endState);
}
