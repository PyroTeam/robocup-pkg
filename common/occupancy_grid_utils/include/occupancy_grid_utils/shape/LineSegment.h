/**
 * \file        LineSegment.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-19
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_LINESEGMENT_H_
#define OCCUPANCY_GRID_UTILS_LINESEGMENT_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class LineSegment : public Shape
{
public:
    LineSegment();
    LineSegment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    ~LineSegment();

    inline void setPoints(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    inline void getPoints(geometry_msgs::Point &p1, geometry_msgs::Point &p2) const;
    virtual void draw(nav_msgs::OccupancyGrid &grid, int max_value=100) override;
protected:
    geometry_msgs::Point m_p1;
	geometry_msgs::Point m_p2;
};


inline void LineSegment::setPoints(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    m_p1 = p1;
    m_p2 = p2;
}

inline void LineSegment::getPoints(geometry_msgs::Point &p1, geometry_msgs::Point &p2) const
{
    p1 = m_p1;
    p2 = m_p2;
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_LINESEGMENT_H_ */
