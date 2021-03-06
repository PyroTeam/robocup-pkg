#include "final_approach/SimFinalApproach.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <final_approach_msg/FinalApproachingAction.h>

#include <cmath>
#include <vector>
#include <list>


SimFinalApproach::SimFinalApproach(std::string name):
	m_as(m_nh, name, boost::bind(&SimFinalApproach::executeCB, this, _1), false)
	, m_actionName(name)
{
	m_as.registerPreemptCallback(boost::bind(&SimFinalApproach::preemptCB, this));
	m_as.start();

	m_pubMvt = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
	m_landmarksSub = m_nh.subscribe("objectDetection/landmarks", 1, &SimFinalApproach::landmarksCallback, this);
}

SimFinalApproach::~SimFinalApproach(void){}

void SimFinalApproach::preemptCB()
{
	ROS_INFO("%s: Preempted", m_actionName.c_str());
	m_as.setPreempted();
}

void SimFinalApproach::landmarksCallback(const deplacement_msg::Machines& msg)
{
	/* XXX : Maybe this vector could be updated only once, because mps position should not change with mocked detecion */

  for (auto &it : msg.landmarks)
  {
    	m_mps.push_back(it.pose);
	}
}

void SimFinalApproach::executeCB(const final_approach_msg::FinalApproachingGoalConstPtr &goal)
{
  m_result.success = true;
  m_result.state = final_approach_msg::FinalApproachingResult::UNKNOWN;
  ROS_INFO("%s: Succeeded", m_actionName.c_str());
  m_as.setSucceeded(m_result);
}
