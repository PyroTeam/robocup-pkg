/**
 * \file        occupancy_grid_utils.cpp
 *
 * \brief       bibliothèque de fonctions de manipulation des nav_msgs::OccupancyGrid
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-11-21
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "occupancy_grid_utils.h"
#include <set>
#include <unordered_set>

namespace occupancy_grid_utils {


/**
 * fonction qui calcule la position dans le vecteur de donnée d'un OccupancyGrid
 * d'un point fourni en coordonnées métrique
 *
 * \param grid la grille
 * \param x abscisse du point en m
 * \param y ordonnée du point en m
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
 * \param x abscisse du point en m
 * \param y ordonnée du point en m
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
 * fonction qui retourne les coordonnées en pixel d'un point donnée en coordonnées métrique
 *
 * \param grid la grille
 * \param x abscisse du point en m
 * \param y ordonnée du point en m
 *
 * \return point en coordonnées pixel
 */
geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, float x, float y)
{

	//TODO redondance de code à regler avec la fonction getCell
	geometry_msgs::Point p;
	p.x = -1;
	p.y = -1;
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
		return p;
	}

	p.y = round((y - y0) / res);
	p.x = round((x - x0) / res);

	return p;

}

/**
 * fonction qui retourne les coordonnées en pixel d'un point donnée en coordonnées métrique
 *
 * \param grid la grille
 * \param p point en coordonnées métriques
 *
 * \return point en coordonnées pixel
 */
geometry_msgs::Point getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p)
{
	return getCellAsPixelCoord(grid, p.x, p.y);
}


geometry_msgs::Pose2D getCellAsPixelCoord(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &pose2d)
{
	geometry_msgs::Point pt = getCellAsPixelCoord(grid, float(pose2d.x), float(pose2d.y));
	geometry_msgs::Pose2D result;
	result.x = pt.x;
	result.y = pt.y;
	result.theta = pose2d.theta;

	return result;
}


/**
 * fonction qui modifie la valeur d'une cellule de la grille
 *
 * \param grid la grille
 * \param x abscisse du point en m
 * \param y ordonnée du point en m
 * \param value valeur à attribuer à la cellule
 *
 */
void setCell(nav_msgs::OccupancyGrid &grid, float x, float y, int value)
{
	geometry_msgs::Point p = getCellAsPixelCoord(grid, x, y);
	setPixelCell(grid, p, value);
}

/**
 * fonction qui modifie la valeur d'une cellule de la grille
 *
 * \param grid la grille
 * \param x abscisse du point en pixel
 * \param y ordonnée du point en pixel
 * \param value valeur à attribuer à la cellule
 *
 */
void setPixelCell(nav_msgs::OccupancyGrid &grid, unsigned int x, unsigned int y, const int& value)
{

	if (x < grid.info.width && y < grid.info.height)
	{
		int valueSaturated = 0;
		if (value < 0)
		{
			valueSaturated = 0;
		}
		else if (value > 100)
		{
			valueSaturated = 100;
		}
		else
		{
			valueSaturated = value;
		}
		grid.data[y * grid.info.width + x] = valueSaturated;

	}

}

/**
 * fonction qui modifie la valeur d'une cellule de la grille
 *
 * \param grid la grille
 * \param p point en coordonnées pixel
 * \param value valeur à attribuer à la cellule
 *
 */
void setPixelCell(nav_msgs::OccupancyGrid &grid, const geometry_msgs::Point &p, const int& value)
{
	setPixelCell(grid, p.x, p.y, value);
}

bool checkRow(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose)
{
	bool found = false;
	double traveledDistance = 0.0;
	geometry_msgs::Pose2D tmp = start;

	while (!found && traveledDistance <= distance)
	{
		if (getCellValue(grid, tmp) < 100)
		{
			found = true;
			foundPose = tmp;
			continue;
		}
		traveledDistance += grid.info.resolution;
		tmp.x += grid.info.resolution;
	}

	return found;
}


bool checkColumn(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &start, double distance, geometry_msgs::Pose2D &foundPose)
{
	bool found = false;
	double traveledDistance = 0.0;
	geometry_msgs::Pose2D tmp = start;

	while (!found && traveledDistance <= distance)
	{
		if (getCellValue(grid, tmp) < 100)
		{
			found = true;
			foundPose = tmp;
			continue;
		}
		traveledDistance += grid.info.resolution;
		tmp.y -= grid.info.resolution;
	}

	return found;
}

bool checkCircle(const geometry_msgs::Pose2D &req, double window, const nav_msgs::OccupancyGrid &grid, geometry_msgs::Pose2D &foundPose)
{
	bool found = false;

	if (checkRow(grid, topLeft(grid, req, window), window, foundPose))
	{
		found = true;
	}
	else if (checkRow(grid, bottomLeft(grid, req, window), window, foundPose))
	{
		found = true;
	}
	else if (checkColumn(grid, topLeft(grid, req, window), window, foundPose))
	{
		found = true;
	}
	else if (checkColumn(grid, topRight(grid, req, window), window, foundPose))
	{
		found = true;
	}

	return found;
}

geometry_msgs::Pose2D topLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window)
{
	geometry_msgs::Pose2D tmp;
	tmp.x = req.x - (window/grid.info.resolution)*grid.info.resolution;
	tmp.y = req.y + (window/grid.info.resolution)*grid.info.resolution;
	tmp.theta = req.theta;

	return tmp;
}

geometry_msgs::Pose2D bottomLeft(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window)
{
	geometry_msgs::Pose2D tmp;
	tmp.x = req.x - (window/grid.info.resolution)*grid.info.resolution;
	tmp.y = req.y - (window/grid.info.resolution)*grid.info.resolution;
	tmp.theta = req.theta;

	return tmp;
}

geometry_msgs::Pose2D topRight(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose2D &req, double window)
{
	geometry_msgs::Pose2D tmp;
	tmp.x = req.x + (window/grid.info.resolution)*grid.info.resolution;
	tmp.y = req.y + (window/grid.info.resolution)*grid.info.resolution;
	tmp.theta = req.theta;

	return tmp;
}


} // namespace occupancy_grid_utils
