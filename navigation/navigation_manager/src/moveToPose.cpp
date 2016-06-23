 /**
 * \file         moveToPose.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise-tissot@polytech-lille.net)
 *               Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2015-04-23
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "navigation_manager/moveToPose.h"
#include <tf/transform_datatypes.h>
#include "deplacement_msg/GeneratePathAction.h"
#include "deplacement_msg/ClosestReachablePoint.h"

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
    actionlib::SimpleActionClient<deplacement_msg::GeneratePathAction> genePathAction("navigation/generatePath", true);

    ROS_INFO("Waiting for path_finder action server to start.");
    genePathAction.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    deplacement_msg::GeneratePathGoal genePathGoal;/*
    //  old silly version with possibly a sad and bad error
    genePathGoal.start.x = m_poseOdom.position.x;
    genePathGoal.start.y = m_poseOdom.position.y;
    genePathGoal.start.theta = tf::getYaw( m_poseOdom.orientation);

*/
    // Client for asking a reachable start position
    ros::ServiceClient client = m_nh.serviceClient<deplacement_msg::ClosestReachablePoint>("path_finder_node/ClosestReachablePoint");
    deplacement_msg::ClosestReachablePoint srv;
    srv.request.currentPosition.x = m_poseOdom.position.x;
    srv.request.currentPosition.y = m_poseOdom.position.y;
    srv.request.currentPosition.theta = tf::getYaw( m_poseOdom.orientation);
    srv.request.window = 0.5;

    if (client.call(srv))
    {
      if (srv.response.found)
      {
        ROS_INFO("Found : (%f,%f,%f)", srv.response.foundPosition.x, srv.response.foundPosition.y, srv.response.foundPosition.theta);
      }
      else
      {
        ROS_INFO("HUMMMMM... ");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service ClosestReachablePoint");
    }

    genePathGoal.start = srv.response.foundPosition;
    genePathGoal.goal =  goal->position_finale;
    genePathGoal.timeout = ros::Duration(10);
    genePathAction.sendGoal(genePathGoal);
    //wait for the action to return
    bool finished_before_timeout = genePathAction.waitForResult(ros::Duration(30.0));

    if (genePathAction.getResult()->result == deplacement_msg::GeneratePathResult::SUCCESS)
    {
        ROS_INFO("Path finder find a path");
    }
    else
    {
        ROS_INFO("Path not found : %d", genePathAction.getResult()->result);
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }

    sleep(1);
    //path Found
    ROS_INFO("Path generated!");
    deplacement_msg::TrackPathGoal tgoal;
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


        //ROS_INFO("In loop with state [%d]", m_trackPathAction.getResult()->status);
        if (!ros::ok())
        {
            isOk = false;
        }
        else if (m_as.isPreemptRequested())
        {
            isOk = false;
		//todo cancel path_track
        }
        else if (m_trackPathAction.getResult()->status == deplacement_msg::TrackPathResult::STATUS_FINISHED)
        {
            isOk = false;
        }
        r.sleep();
    }

    if (m_trackPathAction.getResult()->status == deplacement_msg::TrackPathResult::STATUS_FINISHED)
	  {
        m_result.result = deplacement_msg::MoveToPoseResult::FINISHED;
    }
    else
	  {
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
    }
    m_as.setSucceeded(m_result);

}

// Called once when the goal completes
void MoveToPose::doneCb(const actionlib::SimpleClientGoalState& state,
                        const deplacement_msg::TrackPathResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %d", result->status);
}

// Called once when the goal becomes active
void MoveToPose::activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void MoveToPose::feedbackCb(const deplacement_msg::TrackPathFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length %d", feedback->percentComplete);
    m_pathTrackPercentComplete = feedback->percentComplete;
}

void MoveToPose::PoseCallback(const nav_msgs::Odometry &odom)
{
    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;

    poseIn.header = odom.header;
    poseIn.pose = odom.pose.pose;

    if(!m_tfListener.waitForTransform(
            odom.header.frame_id,
            "map",
            odom.header.stamp,
            ros::Duration(1.0)))
    {
        m_poseOdom = poseIn.pose;
        return;
    }
    m_tfListener.transformPose("/map", poseIn, poseOut);
    m_poseOdom = poseOut.pose;
}


void MoveToPose::DistSensorCallback(const sensor_msgs::PointCloud &sensor)
{
    m_sharpSensor = sensor;
    for (int i = 0 ; i < 9 ; i++)
	  {
  		// ROS_INFO("Données capteur %d : %f %f %f", i, m_sharpSensor.points[i].x, m_sharpSensor.points[i].y, m_sharpSensor.points[i].z);
    }
}
