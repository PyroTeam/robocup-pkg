#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <vector>
#include <list>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include "geometry_utils.h"

class LaserScan
{
public:
    LaserScan();
    ~LaserScan();

    const ros::Time& getTime() const;
    const std::string& getFrame() const;
    const std_msgs::Header& getHeader() const;
    const std::list<geometry_msgs::Point>& getPoints() const;

    void pclCallback(const sensor_msgs::PointCloudConstPtr& pcl);

private:
    std::list<geometry_msgs::Point> m_points;
    std_msgs::Header m_header;
};

#endif
