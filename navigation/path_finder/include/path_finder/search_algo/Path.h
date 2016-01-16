/**
 * \file 		Path.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#ifndef PATH_FINDER_PATH_H_
#define PATH_FINDER_PATH_H_

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "State.h"

/**
 * \class Path
 * \brief Classe représentant un chemin calculé à partir d'un algorithme de recherche
 *
 *
 */
class Path
{
public:
    Path();
    virtual ~Path();

    typedef std::vector<geometry_msgs::PoseStamped>::const_iterator const_iterator ;
    typedef std::vector<geometry_msgs::PoseStamped>::iterator iterator ;
    const_iterator cbegin() const
    {
        return m_poses.cbegin();
    }
    const_iterator cend() const
    {
        return m_poses.cend();
    }
    iterator begin()
    {
        return m_poses.begin();
    }
    iterator end()
    {
        return m_poses.end();
    }
    void clear()
    {
        m_poses.clear();
    }
    bool empty()
    {
        return m_poses.empty();
    }
    std::size_t size()
    {
        return m_poses.size();
    }
    const std::vector<geometry_msgs::PoseStamped> &getPoses() const
    {
        return m_poses;
    }

    void push_front(const geometry_msgs::PoseStamped &pose)
    {
        m_poses.insert(m_poses.begin(), pose);
    }
    void push_back(const geometry_msgs::PoseStamped &pose)
    {
        m_poses.push_back(pose);
    }
    void push_back(const State &state);
    void push_front(const State &state);

    void setSmoothParam(double weightData, double weightSmooth);
    void smooth();

protected:
    std::vector<geometry_msgs::PoseStamped> m_poses;
    double m_weightData;
    double m_weightSmooth;
};

#endif /* PATH_FINDER_PATH_H_ */
