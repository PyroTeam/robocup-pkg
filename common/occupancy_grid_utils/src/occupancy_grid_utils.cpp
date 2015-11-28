/**
 * \file        occupancy_grid_utils.cpp
 *
 * \brief       bibliothèque de fonctions de manipulation des nav_msgs::OccupancyGrid
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-11-21
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "occupancy_grid_utils.h"


namespace occupancy_grid_utils {

/**
 * fonction qui calcule la position dans le vecteur de donnée d'un OccupancyGrid
 * d'un point fourni en coordonnées métrique
 *
 * \param grid la grille
 * \x float abscisse du point en m
 * \y float ordonnée du point en m
 *
 * \return index du tableau data de l'occupancyGrid
 *         -1 si le point n'est pas sur la gridMap
 */
int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
    //code from navigation/path_tracker/src/map.cpp
	//modifié pour ajouter des protections
	float res = grid.info.resolution;
	int width = grid.info.width;
	int height = grid.info.height;
	float x0 = grid.info.origin.position.x;
	float y0 = grid.info.origin.position.y;

	float xMin = x0;
	float xMax = x0 + res*width;
	float yMin = y0;
	float yMax = y0 + res*height;
	if (!(xMin < x && x < xMax) || !(yMin < y && y < yMax))
	{
		return -1;
	}

	int hCell = 0;
	int wCell = 0;
	int cell = 0;

	hCell = round((y - y0) / res);
	wCell = round((x - x0) / res);
	cell = hCell * width + wCell;

	return cell;
}

/**
 * fonction qui retourne la valeur d'un point fourni en coordonnées métrique
 * dans une OccupancyGrid
 *
 * \param grid la grille
 * \x float abscisse du point en m
 * \y float ordonnée du point en m
 *
 * \return valeur du point, 255 si le point n'est pas sur la map
 */
int getCellValue(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
	int cell = 0;
	if ((cell = getCell(grid, x, y)) < 0)
	{
		return 255;
	}
	else
	{
		return grid.data[cell];
	}

}

/**
 * fonction qui retourne la valeur d'un point fourni en coordonnées métrique
 * dans une OccupancyGrid
 *
 * \param grid la grille
 * \p point en coordonnées métrique
 *
 * \return valeur du point, 255 si le point n'est pas sur la map
 */
int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p)
{
    return getCellValue(grid, p.x, p.y);
}

/**
 * fonction qui retourne la valeur d'un point fourni en coordonnées métrique
 * dans une OccupancyGrid
 *
 * \param grid la grille
 * \p pose en coordonnées métrique
 *
 * \return valeur du point, 255 si le point n'est pas sur la map
 */
int getCellValue(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &p)
{
    return getCellValue(grid, p.x, p.y);
}

} // namespace occupancy_grid_utils
