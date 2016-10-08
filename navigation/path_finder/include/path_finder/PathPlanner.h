/**
 * \file 		PathPlanner.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-28
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_PATHPLANNER_H_
#define PATH_FINDER_PATHPLANNER_H_

#include <memory>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include "search_algo/Path.h"
#include "deplacement_msg/GeneratePathAction.h"
#include "search_algo/Path.h"

class Graph;

/**
 * \class PathPlanner
 * \brief Classe de gestion du noeud
 *
 * La classe gère la réponse aux appels de l'action generatePath
 *
 */
class PathPlanner
{
public:
    PathPlanner(const std::shared_ptr<Graph> &graph, std::string name);
    virtual ~PathPlanner();

    const nav_msgs::Path &getPath(bool smoothed = false) const;
protected:
    std::shared_ptr<Graph> m_graph;

    ros::NodeHandle m_nh;
    ros::Publisher m_path_pub;

    actionlib::SimpleActionServer<deplacement_msg::GeneratePathAction> m_path_as;
    deplacement_msg::GeneratePathFeedback m_pathFeedback;
    deplacement_msg::GeneratePathResult m_pathResult;
    void generatePathExecute_callback(const deplacement_msg::GeneratePathGoalConstPtr &goal);

    nav_msgs::Path m_pathFound;
    nav_msgs::Path m_pathSmoothed;
};


#endif /* PATH_FINDER_PATHPLANNER_H_ */
