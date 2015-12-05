#ifndef _TRAIT_IM__SIM_LIGHT_DETECTION__H_
#define _TRAIT_IM__SIM_LIGHT_DETECTION__H_

/*==========  Includes  ==========*/
#include <ros/ros.h>

#include <trait_im_msg/processLightSignalAction.h>
#include <actionlib/server/simple_action_server.h>

#include "trait_im_msg/LightSignal.h"

/*=========================================
=            Class Declaration            =
=========================================*/

class SimLightDetection
{
private:
	// ROS
	ros::NodeHandle m_nh;
	ros::Subscriber m_closestLightSignal_sub;

public:
	SimLightDetection();
	~SimLightDetection();
	bool ok();

private:
	void closestLightSignal_callback(const trait_im_msg::LightSignalConstPtr &msg);

private:
	actionlib::SimpleActionServer<trait_im_msg::processLightSignalAction> m_as;
	std::string m_actionName;
	trait_im_msg::processLightSignalFeedback m_feedback;
	trait_im_msg::processLightSignalResult m_result;

	void goalCB();
	void preemptCB();

    ros::Time m_beginOfProcessing;
    static const float m_timeout = 0.5;
};

/*-----  End of Class Declaration  ------*/

#endif // _TRAIT_IM__SIM_LIGHT_DETECTION__H_
