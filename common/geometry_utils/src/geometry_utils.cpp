/**
 * \file        geometry_utils.cpp
 *
 * \brief       bibliothèque de fonctions de calculs géometriques
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-23
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "geometry_utils.h"


namespace geometry_utils {

double distance(const geometry_msgs::Point &p0, const nav_msgs::Path &path)
{
    // TODO

    return 0.0;
}

geometry_msgs::Point midPoint(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
{
    geometry_msgs::Point m;
    m.x = (p0.x+p1.x)/2.0;
    m.y = (p0.y+p1.y)/2.0;
    m.z = (p0.z+p1.z)/2.0;
    return m;
}

} // namespace geometry_utils
