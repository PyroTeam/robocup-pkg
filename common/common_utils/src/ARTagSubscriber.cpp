/**
 * \file        ARTagSubscriber.h
 * \class       ARTagSubscriber
 * \brief       classe pour s'abonner au topic ar_pose_marker
 * \author      DANEL Thomas (th.danel@gmail.com)
 * \date        2015-06-12
 * \copyright   Association de Robotique de Polytech Lille
 */
#include "common_utils/ARTagSubscriber.h"

namespace common_utils {

ARTagSubscriber::ARTagSubscriber(): m_tfListener(m_nh, ros::Duration(5.0))
{
    m_sub = m_nh.subscribe("computerVision/ar_pose_marker", 1, &ARTagSubscriber::Callback, this);
}

ARTagSubscriber::~ARTagSubscriber()
{

}

void ARTagSubscriber::Callback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
  m_seen_ARTags.clear();

  if(!msg.markers.empty())
  {
    std::string tf_prefix;
    m_nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0){tf_prefix += "/";}


    // les ARTag doivent être publiés dans le repère tower_camera_link du robot
    if (msg.markers.front().header.frame_id.compare(tf_prefix+"tower_camera_link") != 0)
    {
      ROS_ERROR("AR Tags in WRONG FRAME : %s ", msg.markers.front().header.frame_id.c_str());
    }

    // TODO: Filtrage sur le topic pour avoir une information stable sans ID chelou
    // Attention au frame_id, il n'est pas set à la racine, il l'est dans msg.markers.header.frame_id
    // utiliser TF transformPose pour les mettre dans le frame base_link
    // calculer la distance sqrt(transformX*transformX + transformY*transformY)
    // utiliser un threshold pour la distance de vue acceptable (réglagle pour utilisation dans différents noeuds)
    // Docking plutôt 2.0 m
    // Mapping plutôt 5.0 m

    for (auto &it : msg.markers)
    {
      geometry_msgs::PoseStamped poseIn;
      geometry_msgs::PoseStamped poseOut;
      poseIn.header = msg.markers.front().header;
      poseIn.pose = it.pose.pose;

      if(!m_tfListener.waitForTransform(poseIn.header.frame_id, tf_prefix+"base_link", poseIn.header.stamp,ros::Duration(1.0)))
      {
          ROS_ERROR("Unable to get Transform");
          return;
      }
      m_tfListener.transformPose(tf_prefix+"base_link", poseIn, poseOut);

      arTag_t artag;
      artag.pose = poseOut.pose;
      artag.id = it.id;
      m_seen_ARTags.push_back(artag);
    }
    m_stamp = msg.header.stamp;
    m_frame = msg.markers.front().header.frame_id;

  }
}

std::vector<arTag_t> ARTagSubscriber::getARTagsUntil(double threshold)
{
  std::vector<arTag_t> tmp;
  geometry_msgs::Pose zero;
  zero.position.x = 0.0;
  zero.position.y = 0.0;

  for (auto &it : m_seen_ARTags)
  {
    if (geometry_utils::distance(zero, it.pose) <= threshold)
    {
      tmp.push_back(it);
    }
  }

  return tmp;
}

arTag_t ARTagSubscriber::closest(double max)
{
  arTag_t closest;
  geometry_msgs::Pose zero;
  zero.position.x = 0.0;
  zero.position.y = 0.0;
  for (auto &it : m_seen_ARTags)
  {
    double dist = geometry_utils::distance(zero, it.pose);
    if (dist <= max)
    {
      closest = it;
      max = dist;
    }
  }

  return closest;
}

} // namespace common_utils
