#include "moveToPose.h"
#include <tf/transform_datatypes.h>

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
    }

    ros::Time t = ros::Time::now() + ros::Duration(5);
    while (m_last_id != m_path_id && ros::ok() && ros::Time::now() < t){
        r.sleep();
        ros::spinOnce();
    }
    if (ros::Time::now() >= t){
        ROS_INFO("Path generate : Timeout!");
        m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        return;
    }

    if (m_last_id == m_path_id){
        ROS_INFO("Path generated! with id : %d", m_path_id);
        deplacement_msg::TrackPathGoal tgoal;
        tgoal.id = m_last_id;
        m_trackPathAction.sendGoal(tgoal, boost::bind(&MoveToPose::doneCb, this, _1, _2),
        boost::bind(&MoveToPose::activeCb, this),
        boost::bind(&MoveToPose::feedbackCb, this, _1));



        /*bool finished_before_timeout = m_trackPathAction.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout){
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");  */

        while (ros::ok() && !(m_as.isPreemptRequested()) && m_trackPathAction.getResult()->result != deplacement_msg::MoveToPoseResult::FINISHED){

            //algo calculs
            m_as.publishFeedback(m_feedback);
        }
        if (m_trackPathAction.getResult()->result == deplacement_msg::MoveToPoseResult::FINISHED){
            m_result.result = deplacement_msg::MoveToPoseResult::FINISHED;
        }
        else{
            m_result.result = deplacement_msg::MoveToPoseResult::ERROR;
        }
        m_as.setSucceeded(m_result);

    }


    // check that preempt has not been requested by the client
    /*   if (as_.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
    }
    feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
    // publish the feedback
    as_.publishFeedback(feedback_);
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();

    if(success)
    {
        result_.sequence = feedback_.sequence;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    */
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
}

void MoveToPose::PoseCallback(const nav_msgs::Odometry &odom){
    m_pose_odom = odom.pose.pose;
}

void MoveToPose::PathCallback(const pathfinder::AstarPath &path){
    m_path_id = path.id;
}
