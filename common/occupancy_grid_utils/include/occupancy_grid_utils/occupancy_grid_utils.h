/**
 * \file        occupancy_grid_utils.h
 *
 * \brief       bibliothèque de fonctions de manipulation des nav_msgs::OccupancyGrid
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-11-21
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_OCCUPANCY_GRID_UTILS_H_
#define OCCUPANCY_GRID_UTILS_OCCUPANCY_GRID_UTILS_H_

#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y);

int getCellValue(const nav_msgs::OccupancyGrid &grid, float x, float y);
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p);
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &p);

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_OCCUPANCY_GRID_UTILS_H_ */
