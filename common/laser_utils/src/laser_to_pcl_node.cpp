/**
 * \file        simple_cloud_laser_plugin.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-16
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>



class ScanToPCL
{
public:
    ScanToPCL():m_nh()
    {
        m_nh.param<std::string>("simuRobotNamespace", m_tfPrefix, "");
        if (m_tfPrefix.size() != 0)
        {
            m_tfPrefix += "/";
        }
        m_scan_sub = m_nh.subscribe("hardware/scan", 1, &ScanToPCL::scanCallback, this);
        m_pcl_pub = m_nh.advertise<sensor_msgs::PointCloud>("hardware/scan_pcl", 1);
    }

    ~ScanToPCL()
    {

    }

protected:
    ros::NodeHandle m_nh;
    std::string m_tfPrefix;
    laser_geometry::LaserProjection m_projector;
    tf::TransformListener m_listener;

    ros::Subscriber m_scan_sub;
    ros::Publisher m_pcl_pub;

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if(!m_listener.waitForTransform(
            scan_in->header.frame_id,
             m_tfPrefix+"base_link",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0)))
        {
            return;
        }

        sensor_msgs::PointCloud cloud;
        m_projector.transformLaserScanToPointCloud(m_tfPrefix+"base_link", *scan_in, cloud, m_listener);

        m_pcl_pub.publish(cloud);
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_pcl");

    ros::NodeHandle nh;

    ScanToPCL scanToPCL;

    ros::Rate loopRate(20.0);
    ros::spin();
    return 0;
};
