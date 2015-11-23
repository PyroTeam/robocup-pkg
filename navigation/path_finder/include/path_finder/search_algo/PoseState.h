/**
 * \file 		PointState.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#ifndef PATH_FINDER_POSESTATE_H_
#define PATH_FINDER_POSESTATE_H_

#include <geometry_msgs/Pose2D.h>
#include "AStarState.h"

/**
 * \class PoseState
 * \brief Classe concrète représentant un état sous forme d'un geometry_msgs::Pose2D
 *
 *
 */
class PoseState : public AStarState
{
public:
    PoseState() : State()
    {

    }
    virtual ~PoseState()
    {

    }
    void set(double x, double y, double theta)
    {
        m_pose.x = x;
        m_pose.y = y;
        m_pose.theta = theta;
    }
    const geometry_msgs::Pose2D &get() const
    {
        return m_pose;
    }

    virtual bool compare(const State &s) const override
    {
        static double const epsilon = 0.001;
        return (std::abs(m_pose.x - s.get().x) < epsilon &&
                std::abs(m_pose.y - s.get().y) < epsilon &&
                std::abs(m_pose.theta - s.get().theta) < epsilon);
    }

    virtual std::ostream& toStream(std::ostream& os) override
    {
        os << m_pose;
        return os;
    }
protected:
    geometry_msgs::Pose2D m_pose;
};

#endif /* PATH_FINDER_POINTSTATE_H_ */
