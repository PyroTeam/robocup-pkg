/**
 * \file         moveToPose.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise-tissot@polytech-lille.net)
 * \date         2015-04-23
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "navigation_manager/moveToPose.h"
#include <tf/transform_datatypes.h>

const int c_timeOutGenePath = 5;

bool isInZone(float x, float y, float xmin, float xmax, float ymin, float ymax)
{
    if((x < xmin || xmax < x) || (y < ymin || ymax < y))
    {
        return false;
    }
    return true;
}

void MoveToPose::executeCB(const deplacement_msg::MoveToPoseGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Serveur MoveToPose, goal : %f %f %f", goal->position_finale.x, goal->position_finale.y, goal->position_finale.theta);

    // start executing the action
    pathfinder::GeneratePath srv;
    m_lastId++;
    srv.request.id = m_lastId;
    geometry_msgs::Pose2D poseFinale2D = goal->position_finale;
    geometry_msgs::Pose poseFinale;
    poseFinale.position.x = poseFinale2D.x;
    poseFinale.position.y = poseFinale2D.y;
    poseFinale.position.z = goal->position_finale.theta;
    poseFinale.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, poseFinale2D.theta);
    srv.request.Arrivee = poseFinale;
    srv.request.Depart = m_poseOdom;
    srv.request.utilisePositionOdometry = false;

    if (m_generatePathClient.call(srv))
	{
        ROS_INFO("Requete acceptee : %d", srv.response.requeteAcceptee);
    }
    else
	{
        ROS_ERROR("Failed to call service generate path");
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }

    ros::Time t = ros::Time::now() + ros::Duration(c_timeOutGenePath);
    while (m_lastId != m_pathId && ros::ok() && ros::Time::now() < t)
	{
        r.sleep();
        ros::spinOnce();
    }

    //timeout genePath
    if (ros::Time::now() >= t)
	{
        ROS_INFO("Path generate : Timeout!");
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }

    //path Found
    if (m_lastId == m_pathId)
	{
        ROS_INFO("Path generated! with id : %d", m_pathId);
        deplacement_msg::TrackPathGoal tgoal;
        tgoal.id = m_lastId;
        m_trackPathAction.sendGoal(tgoal, boost::bind(&MoveToPose::doneCb, this, _1, _2),
                                          boost::bind(&MoveToPose::activeCb, this),
                                          boost::bind(&MoveToPose::feedbackCb, this, _1));

        bool isOk = true;
        enum PathTrackStatus pathTrackStatus = RUNNING;
        bool obstacleInRange;

		float xmax = 0.4, xmin = 0, ymin = -0.3, ymax = 0.3;

        while (isOk)
        {
            obstacleInRange = false;
            /*if (std::abs(m_sharpSensor.points[0].x) < max && std::abs(m_sharpSensor.points[0].y) < max){
                obstacleInRange = true;
            }
            if (std::abs(m_sharpSensor.points[1].x) < max && std::abs(m_sharpSensor.points[1].y) < max){
                obstacleInRange = true;
            }
            if (std::abs(m_sharpSensor.points[2].x) < max && std::abs(m_sharpSensor.points[2].y) < max){
                obstacleInRange = true;
            }
            if (std::abs(m_sharpSensor.points[7].x) < max && std::abs(m_sharpSensor.points[7].y) < max){
                obstacleInRange = true;
            }
            if (std::abs(m_sharpSensor.points[8].x) < max && std::abs(m_sharpSensor.points[8].y) < max){
                obstacleInRange = true;
            }*/

/*
            if (isInZone(m_sharpSensor.points[0].x, m_sharpSensor.points[0].y, xmin, xmax, ymin, ymax))
            {
                obstacleInRange = true;
            }
            if (isInZone(m_sharpSensor.points[1].x, m_sharpSensor.points[1].y, xmin, xmax, ymin, ymax))
            {
                obstacleInRange = true;
            }
            if (isInZone(m_sharpSensor.points[2].x, m_sharpSensor.points[2].y, xmin, xmax, ymin, ymax))
            {
                obstacleInRange = true;
            }
            if (isInZone(m_sharpSensor.points[7].x, m_sharpSensor.points[7].y, xmin, xmax, ymin, ymax))
            {
                obstacleInRange = true;
            }
            if (isInZone(m_sharpSensor.points[8].x, m_sharpSensor.points[8].y, xmin, xmax, ymin, ymax))
            {
                obstacleInRange = true;
            }


            std::cout << "Obstacle in range :" << obstacleInRange << std::endl;
*/
            if(obstacleInRange && pathTrackStatus == RUNNING)
            {
                m_trackPathAction.cancelGoal();
                pathTrackStatus = PAUSED;
            }
            else if(!obstacleInRange && pathTrackStatus == PAUSED)
            {
                m_trackPathAction.sendGoal(tgoal, boost::bind(&MoveToPose::doneCb, this, _1, _2),
                                                  boost::bind(&MoveToPose::activeCb, this),
                                                  boost::bind(&MoveToPose::feedbackCb, this, _1));
                pathTrackStatus = RUNNING;
            }

            //std::cout << "PathTrack Status = " << pathTrackStatus << std::endl;

            m_feedback.percent_complete = m_pathTrackPercentComplete;
            m_as.publishFeedback(m_feedback);


            if (!ros::ok())
            {
                isOk = false;
            }
            else if (m_as.isPreemptRequested())
            {
                isOk = false;
			//todo cancel path_track
            }
            else if (m_trackPathAction.getResult()->result == deplacement_msg::MoveToPoseResult::FINISHED)
            {
                isOk = false;
            }
        }

        if (m_trackPathAction.getResult()->result == deplacement_msg::MoveToPoseResult::FINISHED)
		{
            m_result.result = deplacement_msg::MoveToPoseResult::FINISHED;
        }
        else
		{
            m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        }
        m_as.setSucceeded(m_result);
    }
    else
    {
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }
}

// Called once when the goal completes
void MoveToPose::doneCb(const actionlib::SimpleClientGoalState& state,
                        const deplacement_msg::TrackPathResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->result);
}

// Called once when the goal becomes active
void MoveToPose::activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void MoveToPose::feedbackCb(const deplacement_msg::TrackPathFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length %d", feedback->percent_complete);
    m_pathTrackPercentComplete = feedback->percent_complete;
}

void MoveToPose::PoseCallback(const nav_msgs::Odometry &odom)
{
    m_poseOdom = odom.pose.pose;
}

void MoveToPose::PathCallback(const pathfinder::AstarPath &path)
{
    m_pathId = path.id;
}

void MoveToPose::DistSensorCallback(const sensor_msgs::PointCloud &sensor)
{
    m_sharpSensor = sensor;
    for (int i = 0 ; i < 9 ; i++)
	{
  		// ROS_INFO("Données capteur %d : %f %f %f", i, m_sharpSensor.points[i].x, m_sharpSensor.points[i].y, m_sharpSensor.points[i].z);
    }
}
