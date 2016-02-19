/**
 * \file         moveToPose.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise-tissot@polytech-lille.net)
 *               Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2015-04-23
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef MOVETOPOSE_H
#define MOVETOPOSE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include "deplacement_msg/MoveToPoseAction.h"
#include "deplacement_msg/TrackPathAction.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

class MoveToPose
{
	private:
	    ros::Subscriber m_pathSub;
	    actionlib::SimpleActionClient<deplacement_msg::TrackPathAction> m_trackPathAction;
	    ros::Subscriber m_odomSub;
	    ros::Subscriber m_sharpSensorSub;

	    sensor_msgs::PointCloud m_sharpSensor;
	    int m_lastId;
	    geometry_msgs::Pose m_poseOdom;
	    int m_pathId;
	    int m_pathTrackPercentComplete;

	    enum PathTrackStatus
	    {
		    RUNNING,
		    PAUSED
	    };

	    void PoseCallback(const nav_msgs::Odometry &odom);
	    void DistSensorCallback(const sensor_msgs::PointCloud &sensor);
	    void doneCb(const actionlib::SimpleClientGoalState& state,
		            const deplacement_msg::TrackPathResultConstPtr& result);
	    void activeCb();
	    void feedbackCb(const deplacement_msg::TrackPathFeedbackConstPtr& feedback);

	protected:
	    ros::NodeHandle m_nh;
	    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
	    actionlib::SimpleActionServer<deplacement_msg::MoveToPoseAction> m_as;
	    std::string m_actionName;
	    // create messages that are used to published feedback/result
	    deplacement_msg::MoveToPoseFeedback m_feedback;
	    deplacement_msg::MoveToPoseResult m_result;
			tf::TransformListener m_tfListener;
	public:
	    MoveToPose(std::string name) : m_as(m_nh, name, boost::bind(&MoveToPose::executeCB, this, _1), false),
	                                   m_actionName(name), m_trackPathAction("navigation/trackPath", true),
																		 m_tfListener(m_nh, ros::Duration(5.0))
	    {
			m_lastId = 0;
			m_pathId = 0;
			m_odomSub = m_nh.subscribe("hardware/odom", 1000, &MoveToPose::PoseCallback, this);
			m_sharpSensorSub = m_nh.subscribe("hardware/distance_sensors", 1000, &MoveToPose::DistSensorCallback, this);
			m_as.start();
	    }

	    ~MoveToPose(void)
	    {
	    }

	    void executeCB(const deplacement_msg::MoveToPoseGoalConstPtr &goal);
};

#endif
