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
#include "topic_tools/MuxSelect.h"

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
    int cptTimeout = 0;

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
    srv.request.currentPosition = m_robotPose.getPose2D();
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

    ros::ServiceClient muxClient = m_nh.serviceClient<topic_tools::MuxSelect>("mux/select");
    topic_tools::MuxSelect muxSrv;
    muxSrv.request.topic = "objectDetection/grid";

    if (muxClient.call(muxSrv))
    {
        ROS_INFO("On utilise la map sans obstacle");
    }
    else
    {
        ROS_ERROR("Failed to call service MuxSelect");
    }

    sleep(1);



    genePathGoal.start = srv.response.foundPosition;
    genePathGoal.goal =  goal->position_finale;
    genePathGoal.timeout = ros::Duration(10);
    genePathAction.sendGoal(genePathGoal);
    //wait for the action to return
    bool finished_before_timeout = genePathAction.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = genePathAction.getState();
        ROS_INFO("Action finished : %s ",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action didn't finish before the time out");
        genePathAction.cancelGoal();
    }

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
    m_isPathTrackEnded = false;
    deplacement_msg::TrackPathGoal tgoal;
    tgoal.command = deplacement_msg::TrackPathGoal::CMD_START;
    m_trackPathAction.sendGoal(tgoal,
        boost::bind(&MoveToPose::doneCb, this, _1, _2),
        boost::bind(&MoveToPose::activeCb, this),
        boost::bind(&MoveToPose::feedbackCb, this, _1));

    bool isOk = true;
    enum PathTrackStatus pathTrackStatus = RUNNING;
    bool obstacleInRange;

    while (isOk)
    {
        obstacleInRange = false;

        if(obstacleInRange && pathTrackStatus == RUNNING)
        {
            tgoal.command = deplacement_msg::TrackPathGoal::CMD_PAUSE;
            m_trackPathAction.sendGoal(tgoal,
                boost::bind(&MoveToPose::doneCb, this, _1, _2),
                boost::bind(&MoveToPose::activeCb, this),
                boost::bind(&MoveToPose::feedbackCb, this, _1));
            pathTrackStatus = PAUSED;
        }
        else if(!obstacleInRange && pathTrackStatus == PAUSED)
        {
            tgoal.command = deplacement_msg::TrackPathGoal::CMD_PAUSE;
            m_trackPathAction.sendGoal(tgoal,
                boost::bind(&MoveToPose::doneCb, this, _1, _2),
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
            //TODO: cancel path_track
        }
        else if(cptTimeout == 3)
        {
            isOk = false;
        }

        if(m_isPathTrackEnded)
        {
            //test les resultats de l'action trackPath
            //TODO:
            switch(m_pathTrackResult.status)
            {
            case deplacement_msg::TrackPathResult::STATUS_UNKNOWN:
                //TODO:
                break;
            case deplacement_msg::TrackPathResult::STATUS_FINISHED:
                //TODO:
                isOk=false;
                break;
            case deplacement_msg::TrackPathResult::STATUS_CANCELLED:
                //TODO:
                break;
            case deplacement_msg::TrackPathResult::STATUS_INTERRUPTED:
                //TODO:
                ROS_INFO("trajectoire interrompue");
                break;
            default:
                ROS_ERROR("L'action TrackPath a retourné un statut non connu");
                isOk = false;
                break;
            }

            switch(m_pathTrackResult.error)
            {
            case deplacement_msg::TrackPathResult::ERR_UNKNOWN:
                //TODO:
                isOk = false;
                break;
            case deplacement_msg::TrackPathResult::ERR_NONE:
                //TODO:
                isOk = false;
                break;
            case deplacement_msg::TrackPathResult::ERR_TRACKING_UNKNOWN:
                //TODO:
                isOk = false;
                break;
            case deplacement_msg::TrackPathResult::ERR_TRACKING_OUT_OF_BOUND:
                //TODO:
                isOk = false;
                break;
            case deplacement_msg::TrackPathResult::ERR_AVOIDANCE_UNKNOWN:
                //TODO:
                isOk = false;
                break;
            case deplacement_msg::TrackPathResult::ERR_AVOIDANCE_UNAVOIDABLE:
                cptTimeout++;
                ROS_INFO("Un obstacle impossible à éviter a été detecté");
                //en cas d'erreur d'évitement on regenere un chemin
                client = m_nh.serviceClient<deplacement_msg::ClosestReachablePoint>("path_finder_node/ClosestReachablePoint");
                srv.request.currentPosition = m_robotPose.getPose2D();
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

                muxClient = m_nh.serviceClient<topic_tools::MuxSelect>("mux/select");
                muxSrv.request.topic = "objectDetection/gridObstacles";


                if (muxClient.call(muxSrv))
                {
                    ROS_INFO("On utilise la map avec obstacle");
                }
                else
                {
                    ROS_ERROR("Failed to call service MuxSelect");
                }

                //sleep(1);

                genePathGoal.start = srv.response.foundPosition;
                genePathGoal.goal =  goal->position_finale;
                genePathGoal.timeout = ros::Duration(10);

                genePathAction.sendGoal(genePathGoal);
                //wait for the action to return
                finished_before_timeout = genePathAction.waitForResult(ros::Duration(30.0));

                if(finished_before_timeout)
                {
                    actionlib::SimpleClientGoalState state = genePathAction.getState();
                    ROS_INFO("Action finished : %s ",state.toString().c_str());
                }
                else
                {
                    ROS_INFO("Action didn't finish before the time out");
                    genePathAction.cancelGoal();
                }

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

                //sleep(1);
                //path Found
                ROS_INFO("Path generated!");
                m_isPathTrackEnded = false;
                tgoal.command = deplacement_msg::TrackPathGoal::CMD_START;
                m_trackPathAction.sendGoal(tgoal,
                    boost::bind(&MoveToPose::doneCb, this, _1, _2),
                    boost::bind(&MoveToPose::activeCb, this),
                    boost::bind(&MoveToPose::feedbackCb, this, _1));

                break;
            case deplacement_msg::TrackPathResult::ERR_AVOIDANCE_UNREACHABLE:
                //TODO:
                break;



            default:
                ROS_ERROR("L'action TrackPath a retourné une erreur non connue");
                isOk = false;
                break;
            }
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
    m_pathTrackResult = *result;
    m_isPathTrackEnded = true;
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

void MoveToPose::DistSensorCallback(const sensor_msgs::PointCloud &sensor)
{
    m_sharpSensor = sensor;
    for (int i = 0 ; i < 9 ; i++)
    {
        // ROS_INFO("Données capteur %d : %f %f %f", i, m_sharpSensor.points[i].x, m_sharpSensor.points[i].y, m_sharpSensor.points[i].z);
    }
}
