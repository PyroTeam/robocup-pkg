/**
 * \file 			finalApproaching.h
 * \class			finalApproaching
 * \brief			classe principale de l approche finale
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef FINALAPPROACHING_H
#define FINALAPPROACHING_H

#include <manager_msg/finalApproachingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

class finalApproaching
{

	protected:
		ros::NodeHandle nh;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<manager_msg::finalApproachingAction> as;
		std::string actionName;
		// create messages that are used to published feedback/result
		manager_msg::finalApproachingFeedback feedback;
		manager_msg::finalApproachingResult result;
		ros::Publisher m_pubMvt;
		ros::Publisher m_markerPub;
		int m_type;
		int m_side;
		int m_parameter;

	public:
		finalApproaching(std::string name) :
		as(nh, name, boost::bind(&finalApproaching::executeCB, this, _1), false),actionName(name)
		{
			as.start();
		}

		~finalApproaching(void);

		void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
		
/**
 *  \brief		avancement de l action
 *  \return		un entier representant l evolution de l action
 */		
		int avancement(int a, int b, int c);
		
/**
 *  \brief		determine la consigne en x (repere laser)
 *  \return		consigne en x
 */
		float objectifX();
		
/**
 *  \brief		determine la consigne en y (repere laser)
 *  \return		consigne en y
 */
		float objectifY();

};


#endif
