/**
 * \file        Paraboloid.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-16
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_PARABOLOID_H_
#define OCCUPANCY_GRID_UTILS_PARABOLOID_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class Paraboloid : public Shape
{
public:
    Paraboloid();
    Paraboloid(const geometry_msgs::Point &p, float a, float b);
    ~Paraboloid();

    inline void setParams(const geometry_msgs::Point &p, float a, float b);
    inline void getParams(geometry_msgs::Point &p, float &a, float &b) const;

    virtual void draw(nav_msgs::OccupancyGrid &grid, int max_value=100) override;
    virtual void draw(deplacement_msg::OccupancyGridFloat &grid, int max_value=100) override;

protected:
    geometry_msgs::Point m_p;
    float m_a;
    float m_b;
};

inline void Paraboloid::setParams(const geometry_msgs::Point &p, float a, float b)
{
    m_p = p;
    m_a = a;
    m_b = b;
}

inline void Paraboloid::getParams(geometry_msgs::Point &p, float &a, float &b) const
{
    p = m_p;
    a = m_a;
    b = m_b;
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_PARABOLOID_H_ */
