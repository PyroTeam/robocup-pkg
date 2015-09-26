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
#define DIST_POINTS_PATH        0.1
#define VIT_ANGLE_MAX           30
#define TIME_OBSTACLE           5
#define DIST_MAX                3
#define EPS                     0.0001
#define OFFSET_ANGLE_AVOIDANCE  0.05

float AvoidanceObstacle::calculDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

float calculNorme(const geometry_msgs::Point &point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}

geometry_msgs::Point AvoidanceObstacle::calculPointsPath(const geometry_msgs::Point &pointD, const geometry_msgs::Point &pointA)
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

std::vector<geometry_msgs::PoseStamped> calculPointsPath2(const geometry_msgs::Point &pointD, const geometry_msgs::Point &pointA)
{

    float xO = pointD.x;
    float yO = pointD.y;
    float angle = atan2(pointA.y - pointD.y, pointA.x - pointD.x);
    std::vector<geometry_msgs::PoseStamped> path;

    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = xO;
    tmp.pose.position.y = yO;
    path.push_back(tmp);

    if (angle > -M_PI/4 && angle < M_PI/4)
    { 
        float a = (pointA.y - pointD.y)/(pointA.x - pointD.x);
        float b = pointA.y - a * pointA.x;
        while (xO < pointA.x)
        {
            xO += DIST_POINTS_PATH;
            yO = a*xO + b;
            tmp.pose.position.x = xO;
            tmp.pose.position.y = yO;
            path.push_back(tmp);   
        }
    }
    else if (angle > M_PI/4 && angle < 3*M_PI/4)
    {
        float a = (pointA.x - pointD.x)/(pointA.y - pointD.y);
        float b = pointA.x - a * pointA.y;        
        while (yO < pointA.y)
        {
            yO += DIST_POINTS_PATH;
            xO = a * yO + b;
            tmp.pose.position.x = xO;
            tmp.pose.position.y = yO;
            path.push_back(tmp);   
        }
    }
    else if (angle < -M_PI/4 && angle > -3*M_PI/4)
    { 
        float a = (pointA.x - pointD.x)/(pointA.y - pointD.y);
        float b = pointA.x - a * pointA.y;        
        while (yO > pointA.y)
        {
            yO -= DIST_POINTS_PATH;
            xO = a * yO + b;
            tmp.pose.position.x = xO;
            tmp.pose.position.y = yO;
            path.push_back(tmp);   
        }
    }
    else // angle > 3*M_PI/4 || angle < -3*M_PI/4 
    {
        float a = (pointA.y - pointD.y)/(pointA.x - pointD.x);
        float b = pointA.y - a * pointA.x;
        while (xO > pointA.x)
        {
            xO -= DIST_POINTS_PATH;
            yO = a*xO + b;
            tmp.pose.position.x = xO;
            tmp.pose.position.y = yO;
            path.push_back(tmp);   
        }
    }
    return path;  
}

/*float AvoidanceObstacle::normaliseAngle(float angle)
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
}*/

/*void AvoidanceObstacle::track(geometry_msgs::Point point, geometry_msgs::Point pointSuiv, geometry_msgs::Pose odom)
{
    float dx = pointSuiv.x - point.x;
    float dy = pointSuiv.y - point.y;
    float ang = atan2(dy, dx);

    float adj = point.x - odom.position.x;
    float opp = point.y - odom.position.y;
    float angle = atan2(opp, adj);
    float yaw = tf::getYaw(odom.orientation);

    float errAngle = (angle - yaw);
    errAngle = normaliseAngle(errAngle);

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
    m_cmdVel.linear.x = 0.2;
    m_cmdVel.linear.y = errAngle/10;
    m_cmdVel.angular.z = vitAngle*1;
    m_cmdVel_pub.publish(m_cmdVel);
}*/

geometry_msgs::Point AvoidanceObstacle::getPointAwayFromObstacle(const std::vector<geometry_msgs::Point> &vectorObstacle, const geometry_msgs::Point &odom)
{
    geometry_msgs::Point pointTarget = vectorObstacle[0];
    geometry_msgs::Point pointStart = vectorObstacle[0];
    float minCosTheta = 1.1;
    for (int i = 0 ; i < vectorObstacle.size() ; i++)
    {
        float scalaire = (pointStart.x - odom.x) * (vectorObstacle[i].x - odom.x)  + (pointStart.y - odom.y) * (vectorObstacle[i].y - odom.y);
        float u = calculDistance(pointStart, odom);
        float v = calculDistance(vectorObstacle[i], odom);
        float cosTheta = scalaire / (u * v);
        //ROS_INFO("Cos theta : %f", cosTheta);

        if (cosTheta < minCosTheta)
        {
            pointTarget = vectorObstacle[i];
            minCosTheta = cosTheta;
        }
    }
    return pointTarget;
}

geometry_msgs::Point transformFrameRobot(const geometry_msgs::Point &point, const geometry_msgs::Point &odom, float cosAlpha, float sinAlpha)
{
    geometry_msgs::Point p;
    p.x = point.x*cosAlpha - point.y*sinAlpha + odom.x;
    p.y = point.x*sinAlpha + point.y*cosAlpha + odom.y;
    return p;
}

geometry_msgs::Point transformFrameGlobal(const geometry_msgs::Point &point, const geometry_msgs::Point &odom, float cosAlpha, float sinAlpha)
{
    geometry_msgs::Point p;
    p.x = point.x*cosAlpha + point.y*sinAlpha - cosAlpha*odom.x - sinAlpha*odom.y;
    p.y = -point.x*sinAlpha + point.y*cosAlpha + sinAlpha*odom.x - cosAlpha*odom.y;
    return p; 
}

bool compareFloat(float x, float y)
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

bool comparePoints(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
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

void cleanPath(std::vector<geometry_msgs::PoseStamped> &points, const geometry_msgs::Point &pointObstacle)
{
    std::vector<geometry_msgs::PoseStamped>::iterator it = points.begin();
    while (it != points.end() && !comparePoints(it->pose.position, pointObstacle))
    {
        it++;
    }
    if (it != points.end())
    {
        points.erase(points.begin(), it);
    }
}

geometry_msgs::Point searchPoint(const std::vector<geometry_msgs::Point> &vectorObstacle, const std::vector<geometry_msgs::PoseStamped> &path)
{
    int i = 0;
    bool end = false;
    bool alreadySeen = false;
    while (i < path.size() && !end)
    {
        int j = 0;
        bool obstacleFound = false;
        while (j < vectorObstacle.size() && !obstacleFound)
        {
            if (!comparePoints(path[i].pose.position, vectorObstacle[j]))
            {
                j++;
            }
            else
            {
                obstacleFound = true;
                alreadySeen = true;
            }
        }
        if (j == vectorObstacle.size() && alreadySeen) // Pas trouvé d'obstacle sur le point du chemin
        {
            end = true;
        }
        else
        {
            i++;
        }
    }
    if (!end)
    {
        ROS_INFO("Erreur point obstacle");
    }
    return path[i].pose.position;
}

void AvoidanceObstacle::avoid(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path, actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> &as, deplacement_msg::TrackPathFeedback &feedback)
{
    m_dataMapObstacle.calculObstacle(odom, path);
    std::vector<geometry_msgs::Point> vectorObstacle = m_dataMapObstacle.getVectorObstacle();
    ROS_INFO("Taille vector obstacles : %d", (int)vectorObstacle.size());

    geometry_msgs::Point pointTarget;
    if (vectorObstacle.size() == 0)
    {
        m_failure = true;
        ROS_INFO("Erreur taille obstacle");
        return;
    }
    /*pointTarget = vectorObstacle[0];
    for (int i = 1 ; i < vectorObstacle.size() ; i++)
    {
        if (calculDistance(m_dataMapObstacle.getPointPathObstacle(), vectorObstacle[i]) > calculDistance(m_dataMapObstacle.getPointPathObstacle(), pointTarget))
        {
            pointTarget = vectorObstacle[i];
        }
    }*/

    pointTarget = getPointAwayFromObstacle(vectorObstacle, odom.position);

    float cosAlpha = (vectorObstacle[0].x - odom.position.x)/calculDistance(vectorObstacle[0], odom.position);
    float sinAlpha = (vectorObstacle[0].y - odom.position.y)/calculDistance(vectorObstacle[0], odom.position);
    geometry_msgs::Point pointTargetRobot = transformFrameGlobal(pointTarget, odom.position, cosAlpha, sinAlpha);
    ROS_INFO("Point target robot : x = %f, y = %f", pointTargetRobot.x, pointTargetRobot.y);

    float theta = atan2(pointTargetRobot.y, pointTargetRobot.x);
    if (theta > 0)
    {
        theta += OFFSET_ANGLE_AVOIDANCE;
    }
    else // theta <= 0
    {
        theta -= OFFSET_ANGLE_AVOIDANCE;
    }
    ROS_INFO("Theta : %f", theta);

    float alpha = atan2(sinAlpha, cosAlpha);
    ROS_INFO("Alpha : %f", alpha);

    geometry_msgs::Point newPointTarget;
    newPointTarget.x = cos(theta)*(calculNorme(pointTargetRobot)+0.5);
    newPointTarget.y = sin(theta)*(calculNorme(pointTargetRobot)+0.5);
    ROS_INFO("New point target : x = %f, y = %f", newPointTarget.x, newPointTarget.y);

    geometry_msgs::Point pointTargetBis;
    pointTargetBis.x = cos(theta)*(calculNorme(pointTargetRobot)+0.5);
    pointTargetBis.y = sin(theta)*(calculNorme(pointTargetRobot)+0.5);
    ROS_INFO("Point target bis : x = %f, y = %f", pointTargetBis.x, pointTargetBis.y);

    pointTarget = transformFrameRobot(newPointTarget, odom.position, cosAlpha, sinAlpha);

    ROS_INFO("Point à atteindre : x = %f, y = %f", pointTarget.x, pointTarget.y);

    float distRobot = calculDistance(odom.position, pointTarget);
    int nbPoints = distRobot/DIST_POINTS_PATH;

    // On construit le chemin jusqu'au point extrême
    geometry_msgs::Point pointD = odom.position;
    geometry_msgs::Point point;
    geometry_msgs::PoseStamped poseStamped;

    m_intermediatePath.clear();
    m_intermediatePath = calculPointsPath2(pointD, pointTarget);
    ROS_INFO("Debut chemin : x = %f, y = %f", pointD.x, pointD.y);
    ROS_INFO("Fin chemin : x = %f, y = %f", pointD.x, pointD.y);
    ROS_INFO("Points intermediatePath : debut x = %f, y = %f, fin x = %f, y = %f", m_intermediatePath[0].pose.position.x, m_intermediatePath[0].pose.position.y, m_intermediatePath.back().pose.position.x, m_intermediatePath.back().pose.position.y);
    static int seq = 0;       
    m_path.header.seq = seq++;
    m_path.header.stamp = ros::Time::now();
    m_path.header.frame_id = "odom";
    m_path.poses = m_intermediatePath;
    m_path_pub.publish(m_path);
    ROS_INFO("Chemin construit");
    ROS_INFO("Nb points chemin : %d", (int)m_intermediatePath.size());

    // On le suit
    int i = 0;
    while (m_intermediatePath.size() > 1 && !m_failure)
    {
        m_dataMapObstacle.calculObstacle(odom, m_intermediatePath);
        if (m_dataMapObstacle.getObstacle())
        {
            m_cmdVel.linear.x = 0;
            m_cmdVel.linear.y = 0;
            m_cmdVel.angular.z = 0;

            m_failure = true;
            ROS_INFO("Echec evitement");

            /*ros::Time t = ros::Time::now() + ros::Duration(TIME_OBSTACLE);
            while (m_dataMapObstacle.getObstacle() && ros::Time::now() < t)
            {
                m_dataMapObstacle.calculObstacle(odom, m_intermediatePath);
            }
            if (ros::Time::now() == t)
            {
                m_failure = true;
            }*/
        }
        else
        {
            m_trackPath.track(m_intermediatePath, odom);
            feedback.distance_path = calculDistance(odom.position, path[0].pose.position);
            as.publishFeedback(feedback);
            i++;
        }
    }
    if (m_failure)
    {
        return;
    }
    ROS_INFO("Premiere partie evitement reussie");
    // Sinon on est arrivé à destination
    // On regarde à présent si le chemin initial est atteignable
    while (!m_successAvoidance && !m_failure)
    {
        bool pathFound = false;
        ROS_INFO("Taille chemin : %d", (int)path.size());
        cleanPath(path, m_dataMapObstacle.getPointPathObstacle());
        geometry_msgs::Point pointEndObstacle = searchPoint(m_dataMapObstacle.getVectorObstacle(), path);
        ROS_INFO("Taille chemin : %d", (int)path.size());
        cleanPath(path, pointEndObstacle);
        ROS_INFO("Taille chemin : %d", (int)path.size());
        std::vector<geometry_msgs::PoseStamped> pathGenerating = path;
        while (pathGenerating.size() > 0 && !pathFound)
        {
            pointD = odom.position;
            m_pointArrival = pathGenerating[0].pose.position;
            ROS_INFO("Point arrival : x = %f, y = %f", m_pointArrival.x, m_pointArrival.y);
            distRobot = calculDistance(odom.position, pathGenerating[0].pose.position);
            nbPoints = distRobot/DIST_POINTS_PATH;
            m_intermediatePath.clear();
            m_intermediatePath = calculPointsPath2(pointD, m_pointArrival);
            /*while (nbPoints > 0)
            {
                point = calculPointsPath2(pointD, m_pointArrival);
                poseStamped.pose.position = point;
                m_intermediatePath.push_back(poseStamped);
                pointD = point;
                nbPoints--;
            }*/           

            m_dataMapObstacle.calculObstacle(odom, m_intermediatePath);
            if (m_dataMapObstacle.getObstacle())
            {
                pathGenerating.erase(pathGenerating.begin());
            }
            else // Pas d'obstacle
            {
                pathFound = true;
            }
        }
        ROS_INFO("Points intermediatePath : debut x = %f, y = %f, fin x = %f, y = %f", m_intermediatePath[0].pose.position.x, m_intermediatePath[0].pose.position.y, m_intermediatePath.back().pose.position.x, m_intermediatePath.back().pose.position.y);
        if (pathGenerating.size() == 0 /*|| calculDistance(odom.position, path[j].pose.position) > DIST_MAX*/)
        {
            ROS_INFO("Obstacle non entierement contourne");
            vectorObstacle = m_dataMapObstacle.getVectorObstacle();
            if (vectorObstacle.size() == 0)
            {
                ROS_INFO("Erreur taille obstacle");
                m_failure = true;
                return;
            }
            /*m_pointArrival = vectorObstacle[0];
            for (int i = 1 ; i < vectorObstacle.size() ; i++)
            {
                if (calculDistance(m_dataMapObstacle.getPointPathObstacle(), vectorObstacle[i]) > calculDistance(m_dataMapObstacle.getPointPathObstacle(), m_pointArrival))
                {
                    m_pointArrival = vectorObstacle[i];
                }
            }
            distRobot = calculDistance(odom.position, m_pointArrival);
            nbPoints = distRobot/DIST_POINTS_PATH;*/
            m_failure = true;
            ROS_INFO("Echec evitement");
        }
        else // On peut atteindre le chemin généré par le pathfinder
        {
            ROS_INFO("On rejoint le chemin");
            ROS_INFO("Points intermediatePath : debut x = %f, y = %f, fin x = %f, y = %f", m_intermediatePath[0].pose.position.x, m_intermediatePath[0].pose.position.y, m_intermediatePath.back().pose.position.x, m_intermediatePath.back().pose.position.y);
            m_almostDone = true;
            /*m_pointArrival = pathGenerating[0].pose.position;
            distRobot = calculDistance(odom.position, pathGenerating[0].pose.position);
            nbPoints = distRobot/DIST_POINTS_PATH;*/
        }
        /*ROS_INFO("Point a atteindre : x = %f, y = %f", m_pointArrival.x, m_pointArrival.y);
        m_intermediatePath.clear();
        while (nbPoints > 0)
        {
            point = calculPointsPath(pointD, m_pointArrival);
            poseStamped.pose.position = point;
            m_intermediatePath.push_back(poseStamped);
            pointD = point;
            nbPoints--;
        }*/
        m_path.header.seq = seq++;
        m_path.header.stamp = ros::Time::now();
        m_path.header.frame_id = "odom";
        m_path.poses = m_intermediatePath;
        m_path_pub.publish(m_path);

        // On le suit
        i = 0;
        while (m_intermediatePath.size() > 1 && !m_failure)
        {
            m_dataMapObstacle.calculObstacle(odom, m_intermediatePath);
            if (m_dataMapObstacle.getObstacle())
            {
                m_cmdVel.linear.x = 0;
                m_cmdVel.linear.y = 0;
                m_cmdVel.angular.z = 0;

                m_failure = true;
                ROS_INFO("Echec evitement");

                /*ros::Time t = ros::Time::now() + ros::Duration(TIME_OBSTACLE);
                while (m_dataMapObstacle.getObstacle() && ros::Time::now() < t)
                {
                    m_dataMapObstacle.calculObstacle(odom, m_intermediatePath);
                }
                if (ros::Time::now() == t)
                {
                    m_failure = true;
                }*/
            }
            else
            {
                m_trackPath.track(m_intermediatePath, odom);
                feedback.distance_path = calculDistance(odom.position, pathGenerating[0].pose.position);
                as.publishFeedback(feedback);
                i++;
            }
        }
        if (m_almostDone && !m_failure)
        {
            /*int k = 0;
            while (k < j)
            {
                path.erase(path.begin());
                k++;
            }*/
            path = pathGenerating;
            m_successAvoidance = true;
            ROS_INFO("Evitement reussi");
        }
    }
}

void AvoidanceObstacle::resetMode()
{ 
    m_failure = false;
    m_successAvoidance = false;
}

bool AvoidanceObstacle::failure()
{
    return m_failure;
}

bool AvoidanceObstacle::successAvoidance()
{
    return m_successAvoidance;
}
