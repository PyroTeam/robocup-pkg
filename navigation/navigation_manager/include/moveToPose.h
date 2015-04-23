#ifndef MOVETOPOSE_H
#define  MOVETOPOSE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "deplacement_msg/MoveToPoseAction.h"
#include "deplacement_msg/TrackPathAction.h"
#include "pathfinder/GeneratePath.h"

#include "nav_msgs/Odometry.h"
#include "pathfinder/AstarPath.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

class MoveToPose
{

private:

    ros::Subscriber m_path_sub;
    actionlib::SimpleActionClient<deplacement_msg::TrackPathAction> m_trackPathAction;
    ros::ServiceClient m_generatePathClient;
    ros::Subscriber m_odom_sub;

    int m_last_id;
    geometry_msgs::Pose m_pose_odom;
    int m_path_id;

    void PoseCallback(const nav_msgs::Odometry &odom);
    void PathCallback(const pathfinder::AstarPath &path);
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const deplacement_msg::TrackPathResultConstPtr& result);
    void activeCb();
    void feedbackCb(const deplacement_msg::TrackPathFeedbackConstPtr& feedback);
    protected:

    ros::NodeHandle m_nh;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<deplacement_msg::MoveToPoseAction> m_as;
    std::string m_action_name;
    // create messages that are used to published feedback/result
    deplacement_msg::MoveToPoseFeedback m_feedback;
    deplacement_msg::MoveToPoseResult m_result;

    public:

    MoveToPose(std::string name) :
    m_as(m_nh, name, boost::bind(&MoveToPose::executeCB, this, _1), false),
    m_action_name(name),
    m_trackPathAction("/trackPath", true)
    {
        m_last_id = 0;
        m_path_id = 0;
        m_odom_sub = m_nh.subscribe("/odom", 1000, &MoveToPose::PoseCallback, this);
        m_path_sub = m_nh.subscribe("/pathFound", 1000, &MoveToPose::PathCallback, this);
        m_generatePathClient = m_nh.serviceClient<pathfinder::GeneratePath>("/generatePath");

        m_as.start();
    }

    ~MoveToPose(void)
    {
    }

    void executeCB(const deplacement_msg::MoveToPoseGoalConstPtr &goal);
};

#endif
