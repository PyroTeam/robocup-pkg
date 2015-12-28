/**
 * \file        BasicGradientModifier.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-28
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#include <list>
#include <utility>
#include "grid_modifier/BasicGradientModifier.h"

namespace occupancy_grid_utils {

void BasicGradientModifier::setParameters(double distance, unsigned int minValue)
{
    if (distance < 0.001) //1mm
    {
        m_distance = 0.05;//5cm arbitrairement
    }
    else
    {
        m_distance = distance;
    }

    if (minValue >100)
    {
        m_minValue = 100;
    }
    else
    {
        m_minValue = minValue;
    }
    m_a = (double(m_minValue) - 100)/m_distance;
}

void BasicGradientModifier::executeImpl(nav_msgs::OccupancyGrid &grid)
{
    const float &resolution = grid.info.resolution;
	const int &width = grid.info.width;
	const int &height = grid.info.height;
	std::vector<signed char> &data = grid.data;

	//on cherche les points bordures
	std::list<std::pair<int,int>> borderPoints;
	for(int x = 1; x < width-1; x++)
	{
		for(int y = 1; y < height-1; y++)
		{
			if (data[y * width + x] == 100)
			{
				if (data[y * width + x-1] != data[y * width + x] ||
					data[y * width + x+1] != data[y * width + x] ||
					data[(y-1) * width + x] != data[y * width + x] ||
					data[(y+1) * width + x] != data[y * width + x] ||
					data[(y-1) * width + x-1] != data[y * width + x] ||
					data[(y+1) * width + x-1] != data[y * width + x] ||
					data[(y-1) * width + x+1] != data[y * width + x] ||
					data[(y+1) * width + x+1] != data[y * width + x])
				{
					borderPoints.push_back(std::make_pair(x, y));
				}
			}
		}
	}

	double sqrt2 = sqrt(2.0);
	for(int x = 0; x < width; x++)
	{
		for(int y = 0; y < height; y++)
		{
			if (data[y * width + x] != 100)
			{
				for (const auto &borderPoint : borderPoints)
				{
					const int &x1 = borderPoint.first;
					const int &y1 = borderPoint.second;
					//double d = sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1))*resolution;
					double dx = std::abs(x - x1);
					double dy = std::abs(y - y1);
					double d = (1 * (dx+dy) + (sqrt2 - 2) * std::min(dx, dy))*resolution;

					signed char value = 0;
					if (d < m_distance)
					{
						value = (signed char)(d * m_a) + 100;
					}

					data[y * width + x] = std::max(value, data[y * width + x]);
				}
			}
		}
	}

}

} // namespace occupancy_grid_utils
