/**
 * \file        GridModifier.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-28
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_GRIDMODIFIER_H_
#define OCCUPANCY_GRID_UTILS_GRIDMODIFIER_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

/**
 * \class GridModifier
 * \brief Classe abstraite pour une interface commune de modificateurs de grid map
 *
 * Les modificateurs peuvent être utiles pour, par exemple, lisser la map ou ajouter
 * des dégradés autours des objets. Ces modificateurs peuvent être chainés pour
 * faciliter leur utilisation.
 */
class GridModifier
{
public:
    GridModifier():m_nextModifier(nullptr)
    {

    }
    virtual ~GridModifier()
    {

    }

    inline void execute(nav_msgs::OccupancyGrid &grid);
    void setNext(const std::shared_ptr<GridModifier> &nextModifier);

protected:
    std::shared_ptr<GridModifier> m_nextModifier;

    virtual void executeImpl(nav_msgs::OccupancyGrid &grid) = 0;
};


inline void GridModifier::execute(nav_msgs::OccupancyGrid &grid)
{
    executeImpl(grid);
    if (m_nextModifier != nullptr)
    {
        m_nextModifier->execute(grid);
    }
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_GRIDMODIFIER_H_ */
