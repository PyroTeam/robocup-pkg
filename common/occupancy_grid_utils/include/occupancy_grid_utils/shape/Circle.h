/**
 * \file        Circle.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_CIRCLE_H_
#define OCCUPANCY_GRID_UTILS_CIRCLE_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class Circle : public Shape
{
public:
    Circle();
    Circle(const geometry_msgs::Point &center, float radius);
    ~Circle();

    virtual void draw(nav_msgs::OccupancyGrid &grid, int max_value=100) override;
    virtual void draw(deplacement_msg::OccupancyGridFloat &grid, int max_value=100) override;
protected:
    geometry_msgs::Point m_center;
    float m_radius;
};

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_CIRCLE_H_ */
