/**
 * \file        FastGradientModifier.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-28
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#include <list>
#include <utility>
#include "grid_modifier/FastGradientModifier.h"

namespace occupancy_grid_utils {

void FastGradientModifier::check(nav_msgs::OccupancyGrid &grid, int x, int y, int x1, int y1, double d)
{
	const int &width = grid.info.width;
	const int &height = grid.info.height;
	std::vector<signed char> &data = grid.data;
;

	double newVal = (double(data[y1 * width + x1]) - 100)/m_a + d;
	double oldVal = (double(data[y * width + x]) - 100)/m_a;
	if(oldVal > newVal)
	{
		data[y * width + x] = newVal*m_a + 100;
	}
}


/**
 * fonction spécifique d'execution effectuant l'ajout des dégradés
 *
 * \param grid la grille sur laquelle on applique les modifications
 *
 */
void FastGradientModifier::executeImpl(nav_msgs::OccupancyGrid &grid)
{
    const float &resolution = grid.info.resolution;
	const int &width = grid.info.width;
	const int &height = grid.info.height;
	std::vector<signed char> &data = grid.data;

	double dx = resolution;
	double dy = resolution;
	double dxy = sqrt(2.0)*resolution;

	for(int y = 1; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			if (x>0)
			{
				check(grid, x, y, x-1, y-1, dxy);
			}
			check(grid, x, y, x, y-1, dy);
			if (x < width-1)
			{
				check(grid, x, y, x+1, y-1, dxy);
			}
		}
		for(int x = 1; x < width; x++)
		{
			check(grid, x, y, x-1, y, dx);
		}
		for(int x = width-2; x >= 0; x--)
		{
			check(grid, x, y, x+1, y, dx);
		}
	}


	for(int y = height-2; y >= 0; y--)
	{
		for(int x = 0; x < width; x++)
		{
			if (x>0)
			{
				check(grid, x, y, x-1, y+1, dxy);
			}
			check(grid, x, y, x, y+1, dy);
			if (x < width-1)
			{
				check(grid, x, y, x+1, y+1, dxy);
			}
		}
		for(int x = 1; x < width; x++)
		{
			check(grid, x, y, x-1, y, dx);
		}
		for(int x = width-2; x >= 0; x--)
		{
			check(grid, x, y, x+1, y, dx);
		}
	}

}

} // namespace occupancy_grid_utils
