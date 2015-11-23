/**
 * \file 		GridMapGraph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <memory>
#include "graph/GridMapGraph.h"

#include "occupancy_grid_utils.h"
namespace grid_utils = occupancy_grid_utils;

GridMapGraph::GridMapGraph() : Graph()
{

}

GridMapGraph::~GridMapGraph()
{

}

void GridMapGraph::setGridMap(const nav_msgs::OccupancyGrid &grid)
{
    m_isInit = true;
    m_grid = grid;
}

void GridMapGraph::getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ)
{
    succ.clear();
    //TODO généraliser ce code
    //On utilise les PointState pour les tests
    const std::shared_ptr<PointState> &currState = std::dynamic_pointer_cast<PointState>(state);
    const geometry_msgs::Point &p = currState->get();

    double resolution = m_grid.info.resolution;
    int cellCost = 0;
    //ROS_INFO_STREAM("Valeur de la case courante (" << p.x << "," << p.y << ") = " << grid_utils::getCellValue(m_grid, p.x, p.y));
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x - resolution, p.y)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x - resolution, p.y);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution);//cellCost
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x, p.y + resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x, p.y + resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution);
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x + resolution, p.y)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x + resolution, p.y);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution);
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x, p.y - resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x, p.y - resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution);
        succ.push_back(nextState);
    }
    static double sqrt_2 = sqrt(2.0);
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x - resolution, p.y - resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x - resolution, p.y - resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution*sqrt_2);
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x - resolution, p.y + resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x - resolution, p.y + resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution*sqrt_2);
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x + resolution, p.y + resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x + resolution, p.y + resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution*sqrt_2);
        succ.push_back(nextState);
    }
    if ((cellCost = grid_utils::getCellValue(m_grid, p.x + resolution, p.y - resolution)) < 100)
    {
        std::shared_ptr<PointState> nextState(new PointState());
        nextState->set(p.x + resolution, p.y - resolution);
        nextState->setPrevState(state);
        nextState->setStepCost(resolution*sqrt_2);
        succ.push_back(nextState);
    }
}

void GridMapGraph::getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred)
{
    getSuccessors(state, pred);
}
