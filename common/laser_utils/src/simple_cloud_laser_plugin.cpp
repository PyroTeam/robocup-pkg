/**
 * \file        simple_cloud_laser_plugin.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-16
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include <pluginlib/class_list_macros.h>
#include "laser_utils/simple_cloud_laser_plugin.h"


SimpleCloudLaser::SimpleCloudLaser()
{

}

SimpleCloudLaser::~SimpleCloudLaser()
{

}

bool SimpleCloudLaser::configure()
{

}

bool SimpleCloudLaser::update( const sensor_msgs::PointCloud & data_in, sensor_msgs::PointCloud& data_out)
{
    data_out = data_in;
}

//PLUGINLIB_EXPORT_PLUGIN(laser_utils::SimpleCloudLaser, filters::FilterBase<sensor_msgs::PointCloud>)

PLUGINLIB_EXPORT_CLASS(SimpleCloudLaser, filters::FilterBase<sensor_msgs::PointCloud>)
