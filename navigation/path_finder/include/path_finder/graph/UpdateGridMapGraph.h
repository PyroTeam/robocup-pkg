/**
 * \file 		UpdateGridMapGraph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_UPDATEGRIDMAPGRAPH_H_
#define PATH_FINDER_UPDATEGRIDMAPGRAPH_H_

#include <memory>
#include <ros/ros.h>
#include "UpdateGraph.h"

class UpdateGridMapGraph : public UpdateGraph
{
public:
    UpdateGridMapGraph();
    virtual ~UpdateGridMapGraph();

};

#endif /* PATH_FINDER_UPDATEGRIDMAPGRAPH_H_ */
