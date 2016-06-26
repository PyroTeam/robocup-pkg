/**
 * \file 		geometry_utils.h
 *
 * \brief       bibliothèque de fonctions de calculs géometriques
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-12-23
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef GEOMETRY_UTILS_GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_GEOMETRY_UTILS_H_

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace geometry_utils {

// En exemple, toutes les fonctions distances possible qui me passaient par la tête,
// je ne le fais pas pour les autres fonctions, ajoutez cells dont vous avez besoin
// au fur et à mesure. En utilisant, si possible, les fonctions existantes pour éviter
// les redondances de code.
inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);
inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Pose &p1);
inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Pose2D &p1);
double distance(const geometry_msgs::Point &p0, const nav_msgs::Path &path);

inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Point &p1);
inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1);
inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Pose2D &p1);
inline double distance(const geometry_msgs::Pose &p0, const nav_msgs::Path &path);

inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Point &p1);
inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Pose &p1);
inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Pose2D &p1);
inline double distance(const geometry_msgs::Pose2D &p0, const nav_msgs::Path &path);

inline double distance(const nav_msgs::Path &path, const geometry_msgs::Point &p0);
inline double distance(const nav_msgs::Path &path, const geometry_msgs::Pose &p0);
inline double distance(const nav_msgs::Path &path, const geometry_msgs::Pose2D &p0);

geometry_msgs::Point midPoint(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);

geometry_msgs::Pose2D changeFrame(const geometry_msgs::Pose2D &p, const tf::StampedTransform &transform);

inline double angle(const geometry_msgs::Point &p0);


inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
{
    return sqrt((p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)*(p0.y - p1.y));
}

inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Pose &p1)
{
    return distance(p0, p1.position);
}

inline double distance(const geometry_msgs::Point &p0, const geometry_msgs::Pose2D &p1)
{
    return sqrt((p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)*(p0.y - p1.y));
}

inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Point &p1)
{
    return distance(p1, p0);
}

inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1)
{
    return distance(p0.position, p1.position);
}

inline double distance(const geometry_msgs::Pose &p0, const geometry_msgs::Pose2D &p1)
{
    return distance(p0.position, p1);
}

inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Point &p1)
{
    return distance(p1, p0);
}

inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Pose &p1)
{
    return distance(p0, p1.position);
}

inline double distance(const geometry_msgs::Pose2D &p0, const geometry_msgs::Pose2D &p1)
{
    return sqrt((p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)*(p0.y - p1.y));
}

inline double distance(const nav_msgs::Path &path, const geometry_msgs::Point &p0)
{
    return distance(p0, path);
}

inline double distance(const nav_msgs::Path &path, const geometry_msgs::Pose &p0)
{
    return distance(p0, path);
}

inline double distance(const nav_msgs::Path &path, const geometry_msgs::Pose2D &p0)
{
    return distance(p0, path);
}

inline double distance(const geometry_msgs::Pose &p0, const nav_msgs::Path &path)
{
    return distance(p0.position, path);
}

inline double distance(const geometry_msgs::Pose2D &p0, const nav_msgs::Path &path)
{
    geometry_msgs::Point p;
    p.x = p0.x;
    p.y = p0.y;
    return distance(p, path);
}

inline double angle(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
{
    return atan2(p1.y-p0.y, p1.x-p1.x);
}

inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle = angle - 2*M_PI;
    }
    while (angle <= -M_PI)
    {
        angle = angle + 2*M_PI;
    }
    return angle;
}

} // namespace geometry_utils

#endif /* GEOMETRY_UTILS_GEOMETRY_UTILS_H_ */
