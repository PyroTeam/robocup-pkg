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

AStarSearch::AStarSearch(const std::shared_ptr<Graph> &graph) : SearchAlgo(graph), m_openList(StateComparison(true))
{

}

AStarSearch::~AStarSearch()
{

}

void AStarSearch::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, std::list<std::shared_ptr<State>> &path)
{
    ROS_INFO_STREAM("PahtFinder : Recherche un chemin : \n Depart \n" << *startState << "\n Arrivee \n" << *endState);
    m_openList.clear();
    m_closedSet.clear();
    m_openList.push(startState);
    while (!m_openList.empty())
    {

        std::shared_ptr<State> currentState = m_openList.top();
        //ROS_INFO_STREAM("Traitement d'un noeud : \n" << *currentState);
        std::shared_ptr<AStarState> currAStarState = std::static_pointer_cast<AStarState>(currentState);
        m_openList.pop();
        if (*currentState == *endState)
        {
            //build path
            ROS_INFO_STREAM("Chemin trouvé");
            path.clear();
            std::shared_ptr<State> temp = currentState;
            while (temp != nullptr)
            {
                path.push_front(temp);
                temp = temp->getPrevState();
            }
            return;
            //return path
        }

        if(m_closedSet.find(currentState) != m_closedSet.end())
        {
            //ROS_INFO_STREAM("Noeud courant déjà dans ClosedSet");
            continue;
        }
        else
        {
            //add current to closedSet
            //ROS_INFO_STREAM("Ajout noeud a ClosedSet");
            m_closedSet.insert(currentState);
            std::list<std::shared_ptr<State>> succ;
            m_graph->getSuccessors(currentState, succ);
            //ROS_INFO_STREAM("Nombre de noeud succ = " << succ.size());
            for (auto &s : succ)
            {
                std::shared_ptr<AStarState> sStar = std::static_pointer_cast<AStarState>(s);
                double gCost = currAStarState->getGCost()+s->getStepCost();
                //test technique tie breaking h = h*(1.0+p) (avec p un poucentage faible)
                //voir http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
                double hCost = m_graph->evaluateHeuristic(*s, *endState) * (1.0 + 1.0/10000.0);
                double fCost = gCost + hCost;
                sStar->setGCost(gCost);
                sStar->setCost(fCost);
                //ROS_INFO_STREAM("Ajout successor à OpenList : \n" << *s);
                m_openList.push(s);
                //usleep(10000);
            }

        }
    }
    //return failure
}
