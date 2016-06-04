/**
 * \file        ComposedShape.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "shape/ComposedShape.h"

namespace occupancy_grid_utils {

ComposedShape::ComposedShape() : Shape()
{

}

ComposedShape::~ComposedShape()
{

}

void ComposedShape::add(std::shared_ptr<Shape> &shape)
{
    m_shapes.push_back(shape);
}

void ComposedShape::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{
    for (auto s : m_shapes)
    {
        s->draw(grid);
    }
}

} // namespace occupancy_grid_utils
