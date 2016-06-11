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

template<typename T>
void createEmptyMap(T &map,
                    const geometry_msgs::Point &size,
                    const geometry_msgs::Point &origin,
                    const std::string &frame_id = "hardware/odom",
                    double resolution = 0.05)
{
	map.header.frame_id = frame_id;
	map.info.origin.position.x = origin.x;
	map.info.origin.position.y = origin.y;
	map.info.origin.position.z = 0;
	map.info.origin.orientation.x = 0;
	map.info.origin.orientation.y = 0;
	map.info.origin.orientation.z = 0;
	map.info.origin.orientation.w = 1;
	map.info.map_load_time = ros::Time::now();
	map.info.resolution = resolution;
	map.info.width = size.x/resolution;
	map.info.height = size.y/resolution;
	map.data.assign(map.info.width * map.info.height, 0);
}

template<typename T>
void createEmptyMap(T &map,
                    nav_msgs::MapMetaData info,
                    const std::string &frame_id = "hardware/odom")
{
	map.header.frame_id = frame_id;
	map.info = info;
	map.info.map_load_time = ros::Time::now();
	map.data.assign(map.info.width * map.info.height, 0);
}

int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y);

int getCellValue(const nav_msgs::OccupancyGrid &grid, float x, float y);
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p){return getCellValue(grid, p.x, p.y);}
inline int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &p){return getCellValue(grid, p.x, p.y);}

void setCell(nav_msgs::OccupancyGrid &grid, float x, float y, int value);
void setPixelCell(nav_msgs::OccupancyGrid &grid, unsigned int x, unsigned int y, const int& value);
void setPixelCell(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p, const int& value);

geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, float x, float y);
geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p);
geometry_msgs::Pose2D getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &pose2d);

bool checkRow(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose);
bool checkColumn(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose);
bool checkCircle(const geometry_msgs::Pose2D &req, double window, const nav_msgs::OccupancyGrid &grid, geometry_msgs::Pose2D &foundPose);

geometry_msgs::Pose2D topLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);
geometry_msgs::Pose2D bottomLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);
geometry_msgs::Pose2D topRight(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window);


/**
 * fonction qui retourne la position métrique d'une case de la grille à partir de son index dans le tableau de données
 *
 * \param grid la grille
 * \param index position dans le tableau grid.data
 *
 * \return point en coordonnée métrique
 */
template<typename T>
geometry_msgs::Point getCellCenter(T &grid, unsigned int index)
{
    geometry_msgs::Point p;

    int a = index/int(grid.info.width);
    int b = index%int(grid.info.width);

    p.y = a*grid.info.resolution + grid.info.origin.position.y;
    p.x = b*grid.info.resolution + grid.info.origin.position.x;

    return p;
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_OCCUPANCY_GRID_UTILS_H_ */
