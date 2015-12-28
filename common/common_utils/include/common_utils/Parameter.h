/**
 * \file 		common_utils/Parameter.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-12-23
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef COMMON_UTILS_PARAMETER_H_
#define COMMON_UTILS_PARAMETER_H_

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

#endif /* COMMON_UTILS_PARAMETER_H_ */
