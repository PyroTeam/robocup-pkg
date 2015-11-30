/**
 * \file 		PathPlanner.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-28
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_PATHPLANNER_H_
#define PATH_FINDER_PATHPLANNER_H_

#include <memory>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "search_algo/Path.h"
#include "deplacement_msg/GeneratePathAction.h"

class Graph;

class PathPlanner
{
public:
    PathPlanner(const std::shared_ptr<Graph> &graph, std::string name);
    virtual ~PathPlanner();

protected:
    std::shared_ptr<Graph> m_graph;
    //actionlib::SimpleActionServer<learning_actionlib::AveragingAction> as_;
    ros::NodeHandle m_nh;
    ros::Publisher m_path_pub;

};


#endif /* PATH_FINDER_PATHPLANNER_H_ */
