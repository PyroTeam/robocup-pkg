/**
 * \file        ComposedShape.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_COMPOSEDSHAPE_H_
#define OCCUPANCY_GRID_UTILS_COMPOSEDSHAPE_H_

#include <vector>

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class ComposedShape : public Shape
{
public:
    ComposedShape();
    virtual ~ComposedShape();

    void add(std::shared_ptr<Shape> &shape);
    virtual void draw(nav_msgs::OccupancyGrid &grid) override;
protected:
    std::vector<std::shared_ptr<Shape>> m_shapes;
};

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_COMPOSEDSHAPE_H_ */
