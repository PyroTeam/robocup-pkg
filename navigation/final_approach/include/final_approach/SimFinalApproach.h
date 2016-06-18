#ifndef __FINAL_APPROACH__SIM_FINAL_APPROACH__H__
#define __FINAL_APPROACH__SIM_FINAL_APPROACH__H__

#include "final_approach/OdomFA.h"

// Ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>

// Ours
#include <final_approach_msg/FinalApproachingAction.h>
#include <deplacement_msg/Landmarks.h>

// Stl
#include <vector>

class SimFinalApproach
{
	public:
		SimFinalApproach(std::string name);
		~SimFinalApproach(void);

		void preemptCB();
		void executeCB(const final_approach_msg::FinalApproachingGoalConstPtr &goal);
		void landmarksCallback(const deplacement_msg::Landmarks::ConstPtr& msg);

	private:
		ros::NodeHandle m_nh;

		/* Action stuffs */
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<final_approach_msg::FinalApproachingAction> m_as;
		std::string m_actionName;
		// create messages that are used to published feedback/result
		final_approach_msg::FinalApproachingFeedback m_feedback;
		final_approach_msg::FinalApproachingResult m_result;

		/* Velocity, markers and plot publishers */
		ros::Publisher m_pubMvt;
		ros::Publisher m_markerPub;
		ros::Publisher m_plot;
		ros::Subscriber m_landmarksSub;

		/* Order infos */
		int m_type;
		int m_side;
		int m_parameter;

		/* MPS Positions */
		std::vector<geometry_msgs::Pose2D> m_mps;

		/* Odom subscriber */
		OdomFA m_odom;
};


#endif // __FINAL_APPROACH__SIM_FINAL_APPROACH__H__
