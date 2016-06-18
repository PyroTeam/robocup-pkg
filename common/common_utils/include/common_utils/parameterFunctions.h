/**
 * \file
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-12-23
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef _COMMON_UTILS__PARAMETERFUNCTIONS__H_
#define _COMMON_UTILS__PARAMETERFUNCTIONS__H_

#include <list>
#include "shape/LineSegment.h"
#include "shape/Rectangle.h"

namespace common_utils {

std::string execProcess(std::string cmd);

void getSetOfParam(ros::NodeHandle &nh, const std::string &paramName, std::set<std::string> &paramSet);

int getParameter(ros::NodeHandle &nh, const std::string &paramName, geometry_msgs::Point &p);
int getParameter(ros::NodeHandle &nh, const std::string &paramName, geometry_msgs::Pose2D &p);
int getParameter(ros::NodeHandle &nh, const std::string &paramName, occupancy_grid_utils::LineSegment &segment);
int getParameter(ros::NodeHandle &nh, const std::string &paramName, occupancy_grid_utils::Rectangle &rectangle);
int getParameter(ros::NodeHandle &nh, const std::string &paramName, std::list<occupancy_grid_utils::LineSegment> &listOfSegments);
int getParameter(ros::NodeHandle &nh, const std::string &paramName, std::list<occupancy_grid_utils::Rectangle> &listOfRectangles);

} // namespace common_utils

#endif /* _COMMON_UTILS__PARAMETERFUNCTIONS__H_ */
