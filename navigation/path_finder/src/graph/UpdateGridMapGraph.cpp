/**
 * \file 		UpdateGridMapGraph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "graph/UpdateGridMapGraph.h"
#include <nav_msgs/OccupancyGrid.h>
#include "occupancy_grid_utils/occupancy_grid_utils.h"

UpdateGridMapGraph::UpdateGridMapGraph(const std::string &topicName, const std::shared_ptr<Graph> &graph):UpdateGraph(topicName, graph)
{
    m_graph_sub = m_nh.subscribe(topicName, 1000, &UpdateGridMapGraph::mapCallback, this);
    m_checkStartService = m_nh.advertiseService("path_finder_node/ClosestReachablePoint", &UpdateGridMapGraph::closestReachablePoint, this);
}

UpdateGridMapGraph::~UpdateGridMapGraph()
{

}

/**
 * Méthode de callback ROS sur un topic fournissant une gridMap
 *
 * \param grid la gridMap reçue
 */
void UpdateGridMapGraph::mapCallback(const nav_msgs::OccupancyGrid &grid)
{
    if (m_graph->isSearchRunning())
    {
        return;
    }
    std::shared_ptr<GridMapGraph> gridMapGraph = std::static_pointer_cast<GridMapGraph>(m_graph);

    gridMapGraph->setGridMap(grid);

    m_grid = grid;
}


bool UpdateGridMapGraph::closestReachablePoint(
  deplacement_msg::ClosestReachablePoint::Request  &req,
  deplacement_msg::ClosestReachablePoint::Response &res)
{
  res.foundPosition = occupancy_grid_utils::checkStartPos(req.currentPosition, req.window, m_grid);
  res.found = true;

  ROS_INFO("request: x=%f, y=%f", req.currentPosition.x, req.currentPosition.y);
  ROS_INFO("sending back response: [x=%f, y=%f]", res.foundPosition.x, res.foundPosition.y);
  return true;
}


geometry_msgs::Pose2D UpdateGridMapGraph::checkStartPos(const geometry_msgs::Pose2D &req, double window, const nav_msgs::OccupancyGrid &grid)
{
	geometry_msgs::Pose2D foundPose;
	foundPose.x = req.x;
	foundPose.y = req.y;
	foundPose.theta = req.theta;

	bool found = false;
	double currentWindow = grid.info.resolution;

	if (getCellValue(grid, req) < 100)
	{
		found = true;
		foundPose = req;
	}

	while (!found && currentWindow < window)
	{
		found = checkCircle(req, currentWindow, grid, foundPose);
		currentWindow += grid.info.resolution;
	}

	return foundPose;
}
