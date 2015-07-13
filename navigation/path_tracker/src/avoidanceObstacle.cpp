/**
 * \file         avoidanceObstacle.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-08
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "avoidanceObstacle.h"

/* Constantes */
#define DIST_POINTS_PATH  0.05
#define VIT_ANGLE_MAX     30
#define TIME_OBSTACLE     5
#define DIST_MAX          3

float AvoidanceObstacle::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

geometry_msgs::Point AvoidanceObstacle::calculPointsPath(geometry_msgs::Point pointD, geometry_msgs::Point pointA)
{
    float a = (pointA.y - pointD.y)/(pointA.x - pointD.x);
    float b = pointA.y - a * pointA.x;
    float xO = pointD.x;
    float yO = pointD.y;
    float delta = 0;
    geometry_msgs::Point point;

    // Intersection entre une droite et un cercle
    delta = 2*(a*(b-yO)-xO)*2*(a*(b-yO)-xO) - 4*(1+a*a)*(xO*xO+(b-yO)*(b-yO)-DIST_POINTS_PATH*DIST_POINTS_PATH);

    if (delta > 0)
    {
        geometry_msgs::Point point1;
        point1.x = (-2*(a*(b-yO)-xO) + sqrt(delta)) / (2*(1+a*a));
        point1.y = a*point1.x + b;
        geometry_msgs::Point point2;
        point2.x = (-2*(a*(b-yO)-xO) + sqrt(delta)) / (2*(1+a*a));
        point2.y = a*point2.x + b;

        if (calculDistance(pointA, point1) < calculDistance(pointA, point2))
        {
            point.x = point1.x;
            point.y = point1.y;
        }
        else
        {
            point.x = point2.x;
            point.y = point2.y;
        }
    }
    else if (delta == 0)
    {
        point.x = (-2*(a*(b-yO)-xO)) / (2*(1+a*a));
        point.y = a*point.x + b;
    }
    else // delta < 0
    {
        ROS_INFO("Erreur points chemin");
    }
    return point;
}

void AvoidanceObstacle::track(geometry_msgs::Point point, geometry_msgs::Point pointSuiv, geometry_msgs::Pose odom)
{
    float dx = pointSuiv.x - point.x;
    float dy = pointSuiv.y - point.y;
    float ang = atan2(dy, dx);

    float adj = point.x - odom.position.x;
    float opp = point.y - odom.position.y;
    float angle = atan2(opp, adj);
    float yaw = tf::getYaw(odom.orientation);

    float errAngle = (angle - yaw);
    errAngle = fmod(errAngle + M_PI, 2 * M_PI) - M_PI;

    float errAnglePointSuiv = (ang - yaw);
    errAnglePointSuiv = fmod(errAnglePointSuiv + M_PI, 2 * M_PI) - M_PI;

    float vitAngle = errAnglePointSuiv * 1;

    if (vitAngle > VIT_ANGLE_MAX)
    {
        vitAngle = VIT_ANGLE_MAX;
    }
    else if (vitAngle < -VIT_ANGLE_MAX)
    {
        vitAngle = -VIT_ANGLE_MAX;
    }
    m_cmdVel.linear.x = 0.2;
    m_cmdVel.linear.y = errAngle/10;
    m_cmdVel.angular.z = vitAngle*1;
    m_cmdVel_pub.publish(m_cmdVel);
}

void AvoidanceObstacle::avoid(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path, actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> &as, deplacement_msg::TrackPathFeedback &feedback)
{
    m_dataMapObstacle.calculObstacle(odom, path[0].pose.position, calculDistance(odom.position, path[0].pose.position));
    m_rightObstacle = m_dataMapObstacle.getObstacleRight();
    m_leftObstacle = m_dataMapObstacle.getObstacleLeft();
    float distRobot = 0;
    float distRobotRight = calculDistance(odom.position, m_rightObstacle);
    float distRobotLeft = calculDistance(odom.position, m_leftObstacle);
    int nbPoints = 0;

    if (distRobotRight <= distRobotLeft) // Le robot est plus proche de la droite de l'obstacle que de la gauche
    {
        m_right = true;
        m_pointArrival = m_rightObstacle;
        distRobot = distRobotRight;
        nbPoints = distRobot/DIST_POINTS_PATH;
    }
    else
    {
        m_right = false;
        m_pointArrival = m_leftObstacle;
        distRobot = distRobotLeft;
        nbPoints = distRobot/DIST_POINTS_PATH;        
    }

    // On construit le chemin jusqu'au point extrême
    geometry_msgs::Point pointD = odom.position;
    geometry_msgs::Point point;
    while (nbPoints > 0)
    {
        point = calculPointsPath(pointD, m_pointArrival);
        m_intermediatePath.push_back(point);
        pointD = point;
        nbPoints--;
    }

    // On le suit
    int i = 0;
    while (i < m_intermediatePath.size() && !m_failure)
    {
        m_dataMapObstacle.calculObstacle(odom, m_intermediatePath[i], calculDistance(m_intermediatePath[i], m_pointArrival));
        if (m_dataMapObstacle.getObstacle())
        {
            m_cmdVel.linear.x = 0;
            m_cmdVel.linear.y = 0;
            m_cmdVel.angular.z = 0;

            ros::Time t = ros::Time::now() + ros::Duration(TIME_OBSTACLE);
            while (m_dataMapObstacle.getObstacle() && ros::Time::now() < t)
            {
                m_dataMapObstacle.calculObstacle(odom, m_intermediatePath[i], calculDistance(m_intermediatePath[i], m_pointArrival));
            }
            if (ros::Time::now() == t)
            {
                m_failure = true;
            }
        }
        else
        {
            geometry_msgs::Point pointSuiv;
            if (m_intermediatePath.size() >= 2)
            {
                pointSuiv = m_intermediatePath[i+1];
            }
            else
            {
                pointSuiv = m_intermediatePath[i];
            }
            track(m_intermediatePath[i], pointSuiv, odom);
            feedback.distance_path = calculDistance(odom.position, path[0].pose.position);
            as.publishFeedback(feedback);
            i++;
        }
    }
    if (m_failure)
    {
        return;
    }
    // Sinon on est arrivé à destination
    // On regarde à présent si le chemin initial est atteignable
    while (!m_successAvoidance && !m_failure)
    {
        int j = 0;
        while (j < path.size() && !m_dataMapObstacle.getObstacle())
        {
            m_dataMapObstacle.calculObstacle(odom, path[j].pose.position, calculDistance(odom.position, path[j].pose.position));
            j++;
        }
        if (j == path.size() || calculDistance(odom.position, path[j].pose.position) > DIST_MAX)
        {
            if (m_right)
            {
                m_pointArrival = m_dataMapObstacle.getObstacleRight();
                distRobot = calculDistance(odom.position, m_rightObstacle);
                nbPoints = distRobot/DIST_POINTS_PATH;
            }
            else // !m_right
            {
                m_pointArrival = m_dataMapObstacle.getObstacleLeft();
                distRobot = calculDistance(odom.position, m_leftObstacle);
                nbPoints = distRobot/DIST_POINTS_PATH;              
            }
        }
        else // On peut atteindre le chemin généré par le pathfinder
        {
            m_almostDone = true;
            m_pointArrival = path[j].pose.position;
            distRobot = calculDistance(odom.position, path[j].pose.position);
            nbPoints = distRobot/DIST_POINTS_PATH;
        }

        while (nbPoints > 0)
        {
            point = calculPointsPath(pointD, m_pointArrival);
            m_intermediatePath.push_back(point);
            pointD = point;
            nbPoints--;
        }

        // On le suit
        i = 0;
        while (i < m_intermediatePath.size() && !m_failure)
        {
            m_dataMapObstacle.calculObstacle(odom, m_intermediatePath[i], calculDistance(m_intermediatePath[i], m_pointArrival));
            if (m_dataMapObstacle.getObstacle())
            {
                m_cmdVel.linear.x = 0;
                m_cmdVel.linear.y = 0;
                m_cmdVel.angular.z = 0;

                ros::Time t = ros::Time::now() + ros::Duration(TIME_OBSTACLE);
                while (m_dataMapObstacle.getObstacle() && ros::Time::now() < t)
                {
                    m_dataMapObstacle.calculObstacle(odom, m_intermediatePath[i], calculDistance(m_intermediatePath[i], m_pointArrival));
                }
                if (ros::Time::now() == t)
                {
                    m_failure = true;
                }
            }
            else
            {
                geometry_msgs::Point pointSuiv;
                if (m_intermediatePath.size() >= 2)
                {
                    pointSuiv = m_intermediatePath[i+1];
                }
                else
                {
                    pointSuiv = m_intermediatePath[i];
                }
                track(m_intermediatePath[i], pointSuiv, odom);
                feedback.distance_path = calculDistance(odom.position, path[0].pose.position);
                as.publishFeedback(feedback);
                i++;
            }
        }
        if (m_almostDone && !m_failure)
        {
            int k = 0;
            while (k < j)
            {
                path.erase(path.begin());
                k++;
            }
            m_successAvoidance = true;
        }
    }
}

bool AvoidanceObstacle::failure()
{
    return m_failure;
}

bool AvoidanceObstacle::successAvoidance()
{
    return m_successAvoidance;
}
