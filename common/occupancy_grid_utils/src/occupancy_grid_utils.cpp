/**
 * \file 		occupancy_grid_utils.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "occupancy_grid_utils.h"


namespace occupancy_grid_utils {

/*int getCell(const nav_msgs::OccupancyGrid &grid, int x, int y)
{

}*/

int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
    //code from navigation/path_tracker/src/map.cpp
	float res = grid.info.resolution;
	int width = grid.info.width;
	float xO = grid.info.origin.position.x;
	float yO = grid.info.origin.position.y;
	int hCell = 0;
	int wCell = 0;
	int cell = 0;

	hCell = round((y - yO) / res);
	wCell = round((x - xO) / res);
	cell = hCell * width + wCell;

	return cell;
}

int getCellValue(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
	return grid.data[getCell(grid, x, y)];
}

int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p)
{
    getCellValue(grid, p.x, p.y);
}

int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &p)
{
    getCellValue(grid, p.x, p.y);
}

} // namespace occupancy_grid_utils
