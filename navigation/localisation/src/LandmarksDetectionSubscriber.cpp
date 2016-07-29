/**
 * \file        LandmarksDetectionSubscriber.h
 * \class       LandmarksDetectionSubscriber
 * \brief       classe pour s'abonner à la détection de machines
 * \author      DANEL Thomas (th.danel@gmail.com)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */

#include "LandmarksDetectionSubscriber.h"
#include <geometry_utils/geometry_utils.h>
#include <common_utils/Zone.h>

#include <Eigen/Dense>

using namespace Eigen;

LandmarksDetectionSubscriber::LandmarksDetectionSubscriber():
    m_tfListener(m_nh, ros::Duration(5.0))
{
    m_sub = m_nh.subscribe("objectDetection/machines", 1, &LandmarksDetectionSubscriber::machinesCallback, this);
}

LandmarksDetectionSubscriber::~LandmarksDetectionSubscriber()
{

}

void LandmarksDetectionSubscriber::machinesCallback(const deplacement_msg::Landmarks& machines)
{
    m_machines.clear();

    static ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    if (machines.landmarks.size() != 0)
    {
        tf::StampedTransform transform;
        try
        {
            m_tfListener.waitForTransform(tf_prefix+"odom",machines.header.frame_id, machines.header.stamp,ros::Duration(1.0));
            m_tfListener.lookupTransform(tf_prefix+"odom", machines.header.frame_id, machines.header.stamp, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }

        for (auto &it : machines.landmarks)
        {
            // Changement de repère
            geometry_msgs::Pose2D center = geometry_utils::changeFrame(it, transform);

            // On enregistre la machine
            if (common_utils::getArea(center) != 0)
            {
                ROS_WARN("Machine en zone %d (%f, %f) enregistree", common_utils::getArea(center), center.x, center.y);
                Vector3d tmp(center.x, center.y, center.theta);
                m_machines.push_back(tmp);
            }
        }
    }
}
