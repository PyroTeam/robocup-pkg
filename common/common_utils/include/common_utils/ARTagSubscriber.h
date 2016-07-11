/**
 * \file        ARTagSubscriber.h
 * \class       ARTagSubscriber
 * \brief       classe pour s'abonner au topic publié par ar_track_alvar
 * \author      DANEL Thomas (th.danel@gmail.com)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */

#ifndef COMMON_UTILS_AR_TAG_SUBSCRIBER_H_
#define COMMON_UTILS_AR_TAG_SUBSCRIBER_H_


#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "geometry_utils/geometry_utils.h"

#include <vector>

namespace common_utils {

typedef struct arTag_s
{
	int id;
	geometry_msgs::Pose pose;
} arTag_t;

class ARTagSubscriber
{
public:
    /* Constructeur */
    ARTagSubscriber();

    /* Destructeur */
    virtual  ~ARTagSubscriber();

    /* Méthodes */
    void Callback(const ar_track_alvar_msgs::AlvarMarkers &msg);
    std::vector<arTag_t> getARTagsUntil(double threshold);
    arTag_t closest(double max);
    bool isEmpty(void) { return m_seen_ARTags.empty();}


private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    tf::TransformListener m_tfListener;

    std::vector<arTag_t> m_seen_ARTags;
    ros::Time m_stamp;
    std::string m_frame;
};

} // namespace common_utils

#endif /* COMMON_UTILS_AR_TAG_SUBSCRIBER_H_ */
