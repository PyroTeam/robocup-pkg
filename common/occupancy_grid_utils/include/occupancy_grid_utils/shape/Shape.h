/**
 * \file        Shape.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_SHAPE_H_
#define OCCUPANCY_GRID_UTILS_SHAPE_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class Shape
{
public:
    Shape();
    virtual ~Shape();

    inline void setName(const std::string &name);
    inline const std::string &getName() const;

    virtual void draw(nav_msgs::OccupancyGrid &grid) = 0;
protected:
    std::string m_name;
};

inline void Shape::setName(const std::string &name)
{
    m_name = name;
}

inline const std::string &Shape::getName() const
{
    return m_name;
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_SHAPE_H_ */
