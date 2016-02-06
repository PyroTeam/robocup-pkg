/**
 * \file        Rectangle.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_RECTANGLE_H_
#define OCCUPANCY_GRID_UTILS_RECTANGLE_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class Rectangle : public Shape
{
public:
    Rectangle();
    Rectangle(const geometry_msgs::Pose2D &pose, const geometry_msgs::Point &size, const float margin=0.0);
    virtual ~Rectangle();

    virtual void draw(nav_msgs::OccupancyGrid &grid, int max_value=100) override;
protected:
     float m_x;
     float m_y;
     float m_theta;
     float m_height;
     float m_width;
     float m_margin;
};

//centre et orientation, taille et marge
void drawRect(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &position, const geometry_msgs::Point &size, float margin)
{
    Rectangle r(position, size, margin);
    r.draw(grid);
}

int drawRect(nav_msgs::OccupancyGrid &grid, float x, float y, float theta, float height, float width, float margin)
{
    geometry_msgs::Pose2D position;
    position.x = x;
    position.y = y;
    position.theta = theta;
    geometry_msgs::Point size;
    size.x = height;
    size.y = width;
    drawRect(grid, position, size, margin);

}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_RECTANGLE_H_ */
