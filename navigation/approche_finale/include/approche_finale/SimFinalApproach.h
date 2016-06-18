#ifndef __APPROCHE_FINALE__SIM_FINAL_APPROACH__HEADER__
#define __APPROCHE_FINALE__SIM_FINAL_APPROACH__HEADER__

// Ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>

// Stl
#include <vector>

// Ours
#include <manager_msg/finalApproachingAction.h>
#include <deplacement_msg/Machines.h>

#include "odomFA.h"

class SimFinalApproach
{
	public:
		SimFinalApproach(std::string name);
		~SimFinalApproach(void);

		void preemptCB();
		void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
    void landmarksCallback(const deplacement_msg::Machines& msg);

	private:
		ros::NodeHandle m_nh;

		/* Action stuffs */
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<manager_msg::finalApproachingAction> m_as;
		std::string m_actionName;
		// create messages that are used to published feedback/result
		manager_msg::finalApproachingFeedback m_feedback;
		manager_msg::finalApproachingResult m_result;

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


#endif // __APPROCHE_FINALE__SIM_FINAL_APPROACH__HEADER__
