/**
 * \file        basic_fusion.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_
#define OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include "deplacement_msg/OccupancyGridFloat.h"

namespace occupancy_grid_utils {


//TODO implémenter les fusions entre tous types d'OccupancyGrid
deplacement_msg::OccupancyGridFloat additiveFusion(const deplacement_msg::OccupancyGridFloat &grid1, nav_msgs::OccupancyGrid &grid2, float maxValue = 100.0);

bool compareGridInfo(nav_msgs::MapMetaData info1, nav_msgs::MapMetaData info2);

//TODO implémenter la conversion inverse
nav_msgs::OccupancyGrid gridConvert(const deplacement_msg::OccupancyGridFloat &grid);


} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_ */
