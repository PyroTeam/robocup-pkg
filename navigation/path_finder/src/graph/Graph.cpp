/**
 * \file 		Graph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "ros/ros.h"
#include "graph/Graph.h"

Graph::Graph():m_isInit(false), m_heuristic(nullptr), m_searchAlgo(nullptr)
{

}

Graph::~Graph()
{

}

/**
 * Accesseur d'affectation du paramètre m_heuristic
 *
 * \param heuristic le pointeur vers l'objet Heuristic
 *
 */
void Graph::setHeuristic(const std::shared_ptr<Heuristic> &heuristic)
{
    m_heuristic = heuristic;
}

/**
 * Accesseur d'affectation du paramètre m_searchAlgo
 *
 * \param searchAlgo le pointeur vers l'objet SearchAlgo
 *
 */
void Graph::setSearchAlgo(const std::shared_ptr<SearchAlgo> &searchAlgo)
{
    m_searchAlgo = searchAlgo;
}

/**
 * Méthode d'évaluation de l'heuristique entre deux noeuds du graph
 *
 * \param startState noeud de départ
 * \param endState noeud d'arrivée
 *
 * \return le résultat du calcul de l'heuristique
 */
double Graph::evaluateHeuristic(State &startState, State &endState)
{
    return m_heuristic->evaluate(startState, endState);
}

/**
 * Méthode de recherche d'un chemin entre deux noeuds du graph
 *
 * \param startState noeud de départ
 * \param endState noeud d'arrivée
 * \param path le chemin généré
 */
void Graph::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, Path &path)
{
    if (m_isInit)
    {
        getClosestNode(startState, startState);
        getClosestNode(endState, endState);
        m_searchAlgo->search(startState, endState, path);
    }
    else
    {
        ROS_ERROR("Tentative de recherche de trajectoire, alors qu'aucune map n'a été reçue.");
    }
}

void Graph::cancelSearch()
{
    m_searchAlgo->cancelSearch();
}

/**
 * Méthode qui retourne le noeud le plus proche dans le graph pour un point fourni
 *
 * \param state noeud fourni
 * \param closestState noeud le plus proche
 */
void Graph::getClosestNode(const std::shared_ptr<State> &state, std::shared_ptr<State> &closestState) const
{
    closestState = state;
}
