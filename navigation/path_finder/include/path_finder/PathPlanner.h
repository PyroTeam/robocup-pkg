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
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include "search_algo/Path.h"
#include "deplacement_msg/GeneratePathAction.h"
#include "path_finder/GeneratePath.h"
#include "search_algo/Path.h"

class Graph;

/**
 * \class PathPlanner
 * \brief Classe de gestion du noeud
 *
 * La classe gère la réponse aux appel de l'action generatePath
 *
 */
class PathPlanner
{
public:
    PathPlanner(const std::shared_ptr<Graph> &graph, std::string name);
    virtual ~PathPlanner();

    const nav_msgs::Path &getPath(bool smoothed = false) const;
    int getPathId() const;
protected:
    std::shared_ptr<Graph> m_graph;
    //actionlib::SimpleActionServer<learning_actionlib::AveragingAction> as_;
    ros::NodeHandle m_nh;
    ros::Publisher m_path_pub;

    //service temporaire : compatibilité avec l'ancien pathFinder
    ros::ServiceServer m_path_srv;
    bool generatePath_callback(path_finder::GeneratePath::Request  &req,
                               path_finder::GeneratePath::Response &res);
    nav_msgs::Path m_pathFound;
    nav_msgs::Path m_pathSmoothed;
    int m_id;
};


#endif /* PATH_FINDER_PATHPLANNER_H_ */
