/**
 * \file        basic_fusion.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-18
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_
#define OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include "deplacement_msg/OccupancyGridFloat.h"

namespace occupancy_grid_utils {

deplacement_msg::OccupancyGridFloat additiveFusion(const deplacement_msg::OccupancyGridFloat &grid1, nav_msgs::OccupancyGrid &grid2);

bool compareGridInfo(nav_msgs::MapMetaData info1, nav_msgs::MapMetaData info2);

nav_msgs::OccupancyGrid gridConvert(const deplacement_msg::OccupancyGridFloat &grid);


} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_BASIC_FUSION_H_ */
