/**
 * \file         dataLaser.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-29
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef DATALASER_H
#define DATALASER_H

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

#include <vector>
#include <cmath>

class DataLaser
{
private:
    bool m_obstacle;
    std::vector<geometry_msgs::Point> m_dataLaser;
    std::vector<geometry_msgs::Point> m_vectorObstacle;

public:
    bool getObstacle();
    void recoverDataLaser(sensor_msgs::LaserScan laserScan);
    float calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    void calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee);

    DataLaser()
    {
        m_obstacle = false;
    }

    ~DataLaser()
    {
    }
};

#endif /* DATALASER_H */
