/**
 * \file        Circle.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_CIRCLE_H_
#define OCCUPANCY_GRID_UTILS_CIRCLE_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
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

void drawCirc(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &center, float radius)
{
    Circle c(center, radius);
    c.draw(grid);
}

void drawCirc(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point32 &center, float radius)
{
    geometry_msgs::Point p;
    p.x = center.x;
    p.y = center.y;
    p.z = center.z;
    Circle c(p, radius);
    c.draw(grid);
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_CIRCLE_H_ */
