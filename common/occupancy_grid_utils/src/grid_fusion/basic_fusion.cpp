/**
 * \file        basic_fusion.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-18
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "grid_fusion/basic_fusion.h"
#include "occupancy_grid_utils/occupancy_grid_utils.h"
namespace occupancy_grid_utils {

const float EPSILON = 0.00001;

deplacement_msg::OccupancyGridFloat additiveFusion(const deplacement_msg::OccupancyGridFloat &grid1, nav_msgs::OccupancyGrid &grid2, float maxValue)
{
    deplacement_msg::OccupancyGridFloat newGrid;
    if (!compareGridInfo(grid1.info, grid2.info))
    {
        //TODO grilles de tailles/resolution/origine différente traiter cette condition correctement
        std::cout << "Grid size mismatch" << std::endl;
        return grid1;
    }

    createEmptyMap(newGrid, grid1.info, grid1.header.frame_id);

    for (unsigned int i=0; i<newGrid.data.size(); ++i)
    {
        newGrid.data[i] = std::min<float>(maxValue, grid1.data[i]+float(grid2.data[i]));
    }
    return newGrid;
}

bool compareGridInfo(nav_msgs::MapMetaData info1, nav_msgs::MapMetaData info2)
{
    return (info1.width == info2.width &&
            info1.height == info2.height &&
            std::abs(info1.origin.position.x - info2.origin.position.x) < EPSILON &&
            std::abs(info1.origin.position.y - info2.origin.position.y) < EPSILON &&
            std::abs(info1.resolution - info2.resolution) < EPSILON);
}


nav_msgs::OccupancyGrid  gridConvert(const deplacement_msg::OccupancyGridFloat &grid)
{
    nav_msgs::OccupancyGrid newGrid;

    createEmptyMap(newGrid, grid.info, grid.header.frame_id);

    for (unsigned int i=0; i<newGrid.data.size(); ++i)
    {
        newGrid.data[i] = std::min<int>(100, int(grid.data[i]));
    }
    return newGrid;
}


} // namespace occupancy_grid_utils
