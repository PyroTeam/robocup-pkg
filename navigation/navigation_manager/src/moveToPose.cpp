#include "navigation_manager/moveToPose.h"
#include <tf/transform_datatypes.h>

const int timeOutGenePath = 5;

void MoveToPose::executeCB(const deplacement_msg::MoveToPoseGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Serveur MoveToPose, goal : %f %f %f", goal->position_finale.x, goal->position_finale.y, goal->position_finale.theta);

    // start executing the action

    pathfinder::GeneratePath srv;
    m_last_id++;
    srv.request.id = m_last_id;
    geometry_msgs::Pose2D pose_finale2D = goal->position_finale;
    geometry_msgs::Pose pose_finale;
    pose_finale.position.x = pose_finale2D.x;
    pose_finale.position.y = pose_finale2D.y;
    pose_finale.position.z = 0;
    pose_finale.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_finale2D.theta);
    srv.request.Arrivee = pose_finale;
    srv.request.Depart = m_pose_odom;
    srv.request.utilisePositionOdometry = false;

    if (m_generatePathClient.call(srv)){
        ROS_INFO("Requete acceptee : %d", srv.response.requeteAcceptee);
    }
    else{
        ROS_ERROR("Failed to call service generate path");
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }

    ros::Time t = ros::Time::now() + ros::Duration(timeOutGenePath);
    while (m_last_id != m_path_id && ros::ok() && ros::Time::now() < t){
        r.sleep();
        ros::spinOnce();
    }
    //timeout genePath
    if (ros::Time::now() >= t){
        ROS_INFO("Path generate : Timeout!");
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        m_as.setAborted(m_result);
        return;
    }

    //path Found
    if (m_last_id == m_path_id){
        ROS_INFO("Path generated! with id : %d", m_path_id);
        deplacement_msg::TrackPathGoal tgoal;
        tgoal.id = m_last_id;
        m_trackPathAction.sendGoal(tgoal, boost::bind(&MoveToPose::doneCb, this, _1, _2),
                                          boost::bind(&MoveToPose::activeCb, this),
                                          boost::bind(&MoveToPose::feedbackCb, this, _1));

        bool isOk = true;
        enum PathTrackStatus pathTrackStatus = RUNNING;
        bool obstacleInRange;



        while (isOk)
        {
            obstacleInRange = false;
            if (m_sharpSensor.points[0].x < 1 && m_sharpSensor.points[0].y < 1){
                obstacleInRange = true;
            }
            if (m_sharpSensor.points[1].x < 1 && m_sharpSensor.points[1].y < 1){
                obstacleInRange = true;
            }
            if (m_sharpSensor.points[2].x < 1 && m_sharpSensor.points[2].y < 1){
                obstacleInRange = true;
            }
            if (m_sharpSensor.points[7].x < 1 && m_sharpSensor.points[7].y < 1){
                obstacleInRange = true;
            }
            if (m_sharpSensor.points[8].x < 1 && m_sharpSensor.points[8].y < 1){
                obstacleInRange = true;
            }

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

            m_feedback.percent_complete = m_pathTrackPercentComplete;
            m_as.publishFeedback(m_feedback);


            if (!ros::ok())
            {
                isOk = false;
            }
            else if (m_as.isPreemptRequested())
            {
                isOk = false;
            }
            else if (m_trackPathAction.getResult()->result != deplacement_msg::MoveToPoseResult::FINISHED)
            {
                isOk = false;
            }

        }

        if (m_trackPathAction.getResult()->result == deplacement_msg::MoveToPoseResult::FINISHED){
            m_result.result = deplacement_msg::MoveToPoseResult::FINISHED;
        }
        else{
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

void MoveToPose::PoseCallback(const nav_msgs::Odometry &odom){
    m_pose_odom = odom.pose.pose;
}

void MoveToPose::PathCallback(const pathfinder::AstarPath &path){
    m_path_id = path.id;
}

void MoveToPose::DistSensorCallback(const sensor_msgs::PointCloud &sensor)
{
    m_sharpSensor = sensor;
    for (int i = 0 ; i < 9 ; i++){
        ROS_INFO("Données capteur %d : %f %f %f", i, m_sharpSensor.points[i].x, m_sharpSensor.points[i].y, m_sharpSensor.points[i].z);
    }
}
