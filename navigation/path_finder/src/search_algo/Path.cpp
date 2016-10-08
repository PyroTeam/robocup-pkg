/**
 * \file 		Path.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include <memory>
#include "search_algo/Path.h"
#include "search_algo/PointState.h"

Path::Path(): m_weightData(0.45), m_weightSmooth(0.35)
{

}

Path::~Path()
{

}

void Path::push_back(const State &state)
{
    //TODO généraliser ce code
    //On utilise les PointState pour les tests
    const PointState &currState = dynamic_cast<const PointState&>(state);
    const geometry_msgs::Point &p = currState.get();
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position = p;
    m_poses.push_back(poseStamped);
}

void Path::push_front(const State &state)
{
    //TODO généraliser ce code
    //On utilise les PointState pour les tests
    const PointState &currState = dynamic_cast<const PointState&>(state);
    const geometry_msgs::Point &p = currState.get();
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position = p;
    m_poses.insert(m_poses.begin(), poseStamped);
}

void Path::setSmoothParam(double weightData, double weightSmooth)
{
    m_weightData = weightData;
    m_weightSmooth = weightSmooth;
}

void Path::smooth()
{
	int pathSize = m_poses.size();
    std::vector<geometry_msgs::PoseStamped> newPath;
    newPath = m_poses;
	double tolerance = 0.000001;
	double change = tolerance+1;
	while (change >= tolerance)
	{
		double aux = 0;
		change = 0.0;
		for (int i=1; i<pathSize-1; i++)
		{
			aux = newPath[i].pose.position.x;
			newPath[i].pose.position.x = newPath[i].pose.position.x + m_weightData * (m_poses[i].pose.position.x - newPath[i].pose.position.x);
	        newPath[i].pose.position.x = newPath[i].pose.position.x + m_weightSmooth * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x-(2.0 * newPath[i].pose.position.x));
	        change =change + std::abs(aux-newPath[i].pose.position.x);

			aux = newPath[i].pose.position.y;
			newPath[i].pose.position.y = newPath[i].pose.position.y + m_weightData * (m_poses[i].pose.position.y - newPath[i].pose.position.y);
	        newPath[i].pose.position.y = newPath[i].pose.position.y + m_weightSmooth * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y-(2.0 * newPath[i].pose.position.y));
	        change = change + std::abs(aux-newPath[i].pose.position.y);
		}
	}
    m_poses = newPath;

}
