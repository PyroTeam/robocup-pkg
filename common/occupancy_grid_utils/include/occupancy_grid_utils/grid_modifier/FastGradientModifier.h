/**
 * \file        FastGradientModifier.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-28
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_FASTGRADIENTMODIFIER_H_
#define OCCUPANCY_GRID_UTILS_FASTGRADIENTMODIFIER_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include <nav_msgs/OccupancyGrid.h>
#include "BasicGradientModifier.h"

namespace occupancy_grid_utils {


/**
 * \class FastGradientModifier
 * \brief Cette classe implémente une version plus rapide de la transformation de distance.
 *
 * Elle est basée sur l'algorithme 8SED de Danielsson
 */
class FastGradientModifier : public BasicGradientModifier
{
public:
    FastGradientModifier(double distance = 0.05, unsigned int minValue = 0):BasicGradientModifier(distance, minValue)
    {

    }
    virtual ~FastGradientModifier()
    {

    }
protected:
    virtual void executeImpl(nav_msgs::OccupancyGrid &grid) override;
private:
    void check(nav_msgs::OccupancyGrid &grid, int x, int y, int x1, int y1, double d);
};

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_FASTGRADIENTMODIFIER_H_ */
