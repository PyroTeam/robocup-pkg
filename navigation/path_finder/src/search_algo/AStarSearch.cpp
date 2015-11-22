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
#include "search_algo/AStarState.h"

AStarSearch::AStarSearch(const std::shared_ptr<Graph> &graph) : SearchAlgo(graph)
{

}

AStarSearch::~AStarSearch()
{

}

void AStarSearch::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState)
{
    ROS_INFO_STREAM("PahtFinder : Recherche un chemin de \n" << *startState << " à \n" << *endState);
    m_openList.clear();
    m_closedSet.clear();
    m_openList.push(startState);
    while (!m_openList.empty())
    {
        std::shared_ptr<State> currentState = m_openList.top();
        std::shared_ptr<AStarState> currAStarState = std::static_pointer_cast<AStarState>(currentState);
        m_openList.pop();
        if (currentState == endState)
        {
            //build path
            //return path
        }
        else if(m_closedSet.find(currentState) != m_closedSet.end())
        {
            continue;
        }
        else
        {
            //add current to closedSet
            m_closedSet.insert(currentState);
            std::list<std::shared_ptr<State>> succ;
            m_graph->getSuccessors(currentState, succ);
            for (auto &s : succ)
            {
                std::shared_ptr<AStarState> sStar = std::static_pointer_cast<AStarState>(s);
                double gCost = currAStarState->getGCost()+s->getStepCost();
                double hCost = m_graph->evaluateHeuristic(*s, *endState);
                double fCost = gCost + hCost;
                sStar->setGCost(gCost);
                sStar->setCost(fCost);
                m_openList.push(s);

            }

        }
    }
    //return failure
}
