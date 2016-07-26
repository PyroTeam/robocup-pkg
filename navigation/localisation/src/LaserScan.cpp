#include "LaserScan.h"
#include <cmath>

LaserScan::LaserScan()
{

}

LaserScan::~LaserScan()
{

}

const ros::Time& LaserScan::getTime() const
{
    return m_header.stamp;
}

const std::string& LaserScan::getFrame() const
{
    return m_header.frame_id;
}

const std_msgs::Header& LaserScan::getHeader() const
{
    return m_header;
}

const std::list<geometry_msgs::Point>& LaserScan::getPoints() const
{
    return m_points;
}

void LaserScan::pclCallback(const sensor_msgs::PointCloudConstPtr& pcl)
{
    m_points.clear();
    m_header = pcl->header;
    for (auto &it : pcl->points)
    {
        geometry_msgs::Point tmp, zero;
        zero.x = zero.y = zero.z = 0.0;

        tmp.x = double(it.x);
        tmp.y = double(it.y);
        tmp.z = double(it.z);

        if (geometry_utils::distance(zero, tmp) <= 5.0)
        {
            m_points.push_back(tmp);
        }
    }
}
