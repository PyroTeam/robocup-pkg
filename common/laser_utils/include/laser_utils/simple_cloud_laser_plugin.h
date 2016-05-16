/**
 * \file        simple_cloud_laser_plugin.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-16
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <filters/filter_base.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud.h>


class SimpleCloudLaser : public filters::FilterBase<sensor_msgs::PointCloud>
{
public:
    SimpleCloudLaser();
    ~SimpleCloudLaser();

    virtual bool configure();
    virtual bool update( const sensor_msgs::PointCloud & data_in, sensor_msgs::PointCloud& data_out);
};

PLUGINLIB_REGISTER_CLASS(SimpleCloudLaserFilter, SimpleCloudLaser, filters::FilterBase<sensor_msgs::PointCloud>);
