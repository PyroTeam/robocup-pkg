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

void createEmptyMap(nav_msgs::OccupancyGrid &map,
                    const geometry_msgs::Point &size,
                    const geometry_msgs::Point &origin,
                    const std::string &frame_id = "hardware/odom",
                    double resolution = 0.05);

int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y);

int getCellValue(const nav_msgs::OccupancyGrid &grid, float x, float y);
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p);
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &p);

void setCell(nav_msgs::OccupancyGrid &grid, float x, float y, int value);
void setPixelCell(nav_msgs::OccupancyGrid &grid, unsigned int x, unsigned int y, const int& value);
void setPixelCell(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p, const int& value);

geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, float x, float y);
geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p);
geometry_msgs::Pose2D getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &pose2d);

bool checkRow(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose);
bool checkColumn(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose);

geometry_msgs::Pose2D topLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);
geometry_msgs::Pose2D bottomLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);
geometry_msgs::Pose2D topRight(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_OCCUPANCY_GRID_UTILS_H_ */
