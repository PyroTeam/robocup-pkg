/**
 * \file         trackPath.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-18
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "trackPath.h"

/* Constantes */
#define VIT_ANGLE_MAX  1
#define EPS            0.0001

bool TrackPath::compareFloat(float x, float y)
{
    if (std::abs(x-y) <= EPS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool TrackPath::comparePoints(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
    if (!compareFloat(point1.x, point2.x) || !compareFloat(point1.y, point2.y) || !compareFloat(point1.z, point2.z))
    {
        return false;
    }
    else
    {
        return true;
    }
}

geometry_msgs::Point TrackPath::closestPoint(const geometry_msgs::Point &segmentStart, const geometry_msgs::Point &segmentStop, const geometry_msgs::Point &point)
{
    geometry_msgs::Point closestPoint;
    float xDelta = segmentStop.x - segmentStart.x;
    float yDelta = segmentStop.y - segmentStart.y;

    if (xDelta == 0 && yDelta == 0)
    {
        closestPoint = segmentStart;
        return closestPoint;
    }

    float u = ((point.x - segmentStart.x) * xDelta + (point.y - segmentStart.y) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

    if (u < 0)
    {
        closestPoint = segmentStart;
    }
    else if (u > 1)
    {
        closestPoint = segmentStop;
    }
    else
    {
        closestPoint.x = segmentStart.x + u * xDelta;
        closestPoint.y = segmentStart.y + u * yDelta;
    }
    return closestPoint;
}

float TrackPath::normaliseAngle(float angle)
{
    float angleNormalise = angle;
    while (angleNormalise <= -M_PI)
    {
        angleNormalise += 2 * M_PI;
    }
    while (angleNormalise > M_PI)
    {
        angleNormalise -= 2 * M_PI;
    }
    return angleNormalise;
}

void TrackPath::track(std::vector<geometry_msgs::PoseStamped> &points, const geometry_msgs::Pose &odom)
{
    geometry_msgs::Point pose;
    pose.x = odom.position.x;
    pose.y = odom.position.y;
    geometry_msgs::Point pointObjectif;
    geometry_msgs::Point closest;
    geometry_msgs::Point start;
    geometry_msgs::Point stop;
    geometry_msgs::Point pointSuiv;
    float finalAngle = 0;


    if (points.size() != 0)
    {
    	m_stopRobot = false;
        m_targetReached = false;
     	finalAngle = tf::getYaw(points.back().pose.orientation);
    }
    pointObjectif = points[0].pose.position;
    for (int i = 0 ; i < points.size() ; i++)
    {
        if (points.size() >= 1)
        {
            start = points[0].pose.position;
            if (points.size() >= 2)
            {
                stop = points[1].pose.position;
                closest = closestPoint(start, stop, pose);
                if (comparePoints(closest, stop))
                {
                    points.erase(points.begin());
                    continue;
                }
                else
                {
                    break;
                }
            }
            else
            {
                closest = start;
                m_targetReached = true;
            }
        }
    }

    // Distance d'avance
    float distAvance = 0.1;

    // Point d'avance
    geometry_msgs::Point pointAvance;

    if (points.size() >= 2)
    {
        pointSuiv = points[1].pose.position;
    }
    else
    {
        pointSuiv = points[0].pose.position;
    }

    float dx = pointSuiv.x - closest.x;
    float dy = pointSuiv.y - closest.y;
    float ang = atan2(dy, dx);

    pointAvance.x = closest.x + distAvance * cos(ang);
    pointAvance.y = closest.y + distAvance * sin(ang);

    m_pointArrivee = pointAvance;

	float angle =0;
    // Rejoindre le point d'avance
    if (!m_targetReached)
    {
	    float adj = pointAvance.x - pose.x;
	    float opp = pointAvance.y - pose.y;
	    angle = atan2(opp, adj);    	
    }
    // Target orientation
    else
    {
    	angle = finalAngle;
    	// ROS_DEBUG_ONCE("Targeted orientation : %f", angle);
    	ROS_INFO("Targeted orientation : %f", angle);
    }

    float yaw = tf::getYaw(odom.orientation);

    float errAngle = (angle - yaw);
    errAngle = normaliseAngle(errAngle);

    /* Check if orientation reached */
    #define FINAL_ORIENTATION_TOLERANCE_RAD	0.1
    if (m_targetReached && fabs(errAngle) < FINAL_ORIENTATION_TOLERANCE_RAD)
    {
   		m_stopRobot = true;
    }
    #undef FINAL_ORIENTATION_TOLERANCE_RAD

    float errAnglePointSuiv = (ang - yaw);
    errAnglePointSuiv = normaliseAngle(errAnglePointSuiv);

    float vitAngle = errAnglePointSuiv * 1;

    if (vitAngle > VIT_ANGLE_MAX)
    {
        vitAngle = VIT_ANGLE_MAX;
    }
    else if (vitAngle < -VIT_ANGLE_MAX)
    {
        vitAngle = -VIT_ANGLE_MAX;
    }

    /* Follow path until target point */
    if (!m_targetReached)
    {
        m_cmdVel.linear.x = 0.2;
        m_cmdVel.linear.y = errAngle/10;
        m_cmdVel.angular.z = vitAngle*1;
    }
    /* Target reached, adjust angle */
    else if (!m_stopRobot)
    {
        m_cmdVel.linear.x = 0;
        m_cmdVel.linear.y = 0;
        m_cmdVel.angular.z = errAngle*1;
    }
    /* Target and orientation reached, finish tracking */
    else
    {
        m_cmdVel.linear.x = 0;
        m_cmdVel.linear.y = 0;
        m_cmdVel.angular.z = 0;

	    m_success = true;
    }
    m_cmdVel_pub.publish(m_cmdVel);
}

bool TrackPath::success()
{
    return m_success;
}

bool TrackPath::failure()
{
    return m_failure;
}

void TrackPath::resetState()
{
    m_success = false;
    m_failure = false;
}

geometry_msgs::Point TrackPath::getPointArrivee()
{
    return m_pointArrivee;
}
