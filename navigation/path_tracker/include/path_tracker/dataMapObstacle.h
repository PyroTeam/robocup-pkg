/**
 * \file         dataMapObstacle.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-09
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef DATAMAPOBSTACLE_H
#define DATAMAPOBSTACLE_H

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <vector>

class DataMapObstacle
{
private:
    ros::Subscriber m_grid_sub;
    ros::Publisher m_pointCloud_pub;
    bool m_obstacle;
    geometry_msgs::Point m_pointObstacle;
    nav_msgs::OccupancyGrid m_grid;
    bool m_receiveGrid;
    std::vector<geometry_msgs::Point> m_vectorObstaclePoints;
    std::vector<int> m_vectorObstacle;
    sensor_msgs::PointCloud m_pointCloud;

    ros::NodeHandle m_nh;

    void gridCallback(const nav_msgs::OccupancyGrid &grid);

public:
    nav_msgs::OccupancyGrid getGridObstacle();
    bool getObstacle();
    geometry_msgs::Point getPointPathObstacle();
    std::vector<geometry_msgs::Point> getVectorObstacle();
    geometry_msgs::Point getPoint(int cell, const nav_msgs::OccupancyGrid &grid);
    //void getPointsMap(const nav_msgs::OccupancyGrid &grid);
    int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y);
    float calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    void calculObstacle(const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path);

    DataMapObstacle()
    {
        m_pointCloud_pub = m_nh.advertise<sensor_msgs::PointCloud>("pointCloudObstacle", 1000);
        m_grid_sub = m_nh.subscribe("/gridObstacles", 1000, &DataMapObstacle::gridCallback, this);
        m_obstacle = false;
        m_receiveGrid = false;
    }
    ~DataMapObstacle()
    {
    }
};

#endif /* DATAMAPOBSTACLE_H */
