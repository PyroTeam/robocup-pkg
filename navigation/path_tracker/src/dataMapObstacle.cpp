/**
 * \file         dataMapObstacle.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-09
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "dataMapObstacle.h"

/* Constantes */
#define LIMIT_POINTS_PATH 20

nav_msgs::OccupancyGrid DataMapObstacle::getGridObstacle()
{
    return m_grid;
}

bool DataMapObstacle::getObstacle()
{
    return m_obstacle;
}

geometry_msgs::Point DataMapObstacle::getPointPathObstacle()
{
    return m_pointObstacle;
}

std::vector<geometry_msgs::Point> DataMapObstacle::getVectorObstacle()
{
    return m_vectorObstaclePoints;
}

float DataMapObstacle::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

geometry_msgs::Point DataMapObstacle::getPoint(int cell, const nav_msgs::OccupancyGrid &grid)
{
	float res = grid.info.resolution;
	int width = grid.info.width;
	float xO = grid.info.origin.position.x;
	float yO = grid.info.origin.position.y;
    int h = 0;
    int w = 0;
    float x = 0;
    float y = 0;
    geometry_msgs::Point point;

    while (((cell - w) % width) != 0)
    {
        w++;
    }
    h = (cell - w) / width;

    x = w * res + xO;
    y = h * res + yO;
    point.x = x;
    point.y = y;
    //ROS_INFO("Point : x = %f, y = %f, h = %d; w = %d", x, y, h, w);

    return point;
}

/*void DataMapObstacle::getPointsMap(const nav_msgs::OccupancyGrid &grid)
{
    geometry_msgs::Point point;
    for (int i = 0 ; i < grid.info.width * grid.info.height ; i++)
    {
        if (grid.data[i] != 0)
        {
            //ROS_INFO("Point %d", i);
            point = getPoint(i, grid);
            m_pointsGrid.push_back(point);
        }
    }
}*/

int DataMapObstacle::getCell(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
	float res = grid.info.resolution;
	int width = grid.info.width;
	float xO = grid.info.origin.position.x;
	float yO = grid.info.origin.position.y;
	int hCell = 0;
	int wCell = 0;
	int cell = 0;

	hCell = round((y - yO) / res);
	wCell = round((x - xO) / res);
	cell = hCell * width + wCell;

	return cell;
}

void DataMapObstacle::calculObstacle(const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path)
{
    m_vectorObstacle.clear();
    m_vectorObstaclePoints.clear();
    m_obstacle = false;
    int cell = 0;
    int width = m_grid.info.width;
    int height = m_grid.info.height;
    bool high = false;
    bool low = false;
    bool left = false;
    bool right = false;
    std::vector<int> tmp;
    if (path.size() >= LIMIT_POINTS_PATH)
    {
        //ROS_INFO("Test");
        int i = 0;
        while (i < LIMIT_POINTS_PATH && !m_obstacle)
        {
            cell = getCell(m_grid, path[i].pose.position.x, path[i].pose.position.y);
            if (m_grid.data[cell] != 0) // Case noircie
            {
                //ROS_INFO("Obstacle");
                m_obstacle = true;
                m_pointObstacle = path[i].pose.position;
                m_vectorObstacle.push_back(cell);
            }
            i++;
        }
        if (!m_obstacle)
        {
            return;
        }
        else
        {
            i = 0;
            while (i < m_vectorObstacle.size())
            {
                //ROS_INFO("i = %d", i);
                cell = m_vectorObstacle[i];
                /*if (cell < width)
                {
                    high = true;
                }
                if (cell >= (width*(height-1)))
                {
                    low = true;
                }
                if ((cell%width) == 0)
                {
                    left = true;
                }
                if (((cell+1)%width) == 0)
                {
                    right = true;
                }
                if (!high && !low && !left && !right)
                {*/
                    tmp.push_back(cell - width - 1);
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - width + 1);
                    tmp.push_back(cell - 1);
                    tmp.push_back(cell + 1);
                    tmp.push_back(cell + width - 1);
                    tmp.push_back(cell + width);
                    tmp.push_back(cell + width + 1);
                /*}
                else if (high && !left && !right)
                {
                    tmp.push_back(cell - 1);
                    tmp.push_back(cell + 1);
                    tmp.push_back(cell + width - 1);
                    tmp.push_back(cell + width);
                    tmp.push_back(cell + width + 1);
                }
                else if (right && !high && !low)
                {
                    tmp.push_back(cell - width - 1);
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - 1);
                    tmp.push_back(cell + width - 1);
                    tmp.push_back(cell + width);    
                }
                else if (low && !left && !right)
                {
                    tmp.push_back(cell - width - 1);
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - width + 1);
                    tmp.push_back(cell - 1);
                    tmp.push_back(cell + 1);
                }
                else if (left && !high && !low)
                {
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - width + 1);
                    tmp.push_back(cell + 1);
                    tmp.push_back(cell + width);
                    tmp.push_back(cell + width + 1);
                }
                else if (high && right)
                {
                    tmp.push_back(cell - 1);
                    tmp.push_back(cell + width - 1);
                    tmp.push_back(cell + width);
                }
                else if (right && low)
                {
                    tmp.push_back(cell - width - 1);
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - 1);
                }
                else if (low && left)
                {
                    tmp.push_back(cell - width);
                    tmp.push_back(cell - width + 1);
                    tmp.push_back(cell + 1);
                }
                else if (left && high)
                {
                    tmp.push_back(cell + 1);
                    tmp.push_back(cell + width);
                    tmp.push_back(cell + width + 1);
                }*/
                int j = 0;
                while (j < tmp.size())
                {
                    if (m_grid.data[tmp[j]] != 0) // Case noircie
                    {
                        int k = 0;
                        while (k < m_vectorObstacle.size() && tmp[j] != m_vectorObstacle[k])
                        {
                            k++;
                        }
                        if (k == m_vectorObstacle.size())
                        {
                            m_vectorObstacle.push_back(tmp[j]);
                        }
                    }
                    j++;
                }
                tmp.clear();
                i++;
            }
            m_pointCloud.points.clear();
            for (int i = 0 ; i < m_vectorObstacle.size() ; i++)
            {
                geometry_msgs::Point point = getPoint(m_vectorObstacle[i], m_grid);
                m_vectorObstaclePoints.push_back(point);
                geometry_msgs::Point32 p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.1;
                m_pointCloud.points.push_back(p);
                //ROS_INFO("Point obstacle : x = %f, y = %f", m_vectorObstaclePoints[i].x, m_vectorObstaclePoints[i].y);
            }
            static int seq = 0;
            m_pointCloud.header.seq = seq++;
            m_pointCloud.header.stamp = ros::Time::now();
            m_pointCloud.header.frame_id = "odom";
            m_pointCloud_pub.publish(m_pointCloud);
        }
    }
}

/*void DataMapObstacle::calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee, float distObstacle)
{
    m_obstacle = false;
    m_lengthObstacle = 0;
    geometry_msgs::Point pointTmp;
    if (m_vectorObstacle.size() != 0)
    {
        m_vectorObstacle.clear();
    }

    while (!m_receiveGrid)
    {
    }
    getPointsMap(m_grid);
    if (m_pointsGrid.size() != 0)
    {
        int i = 1;
        while (i < m_pointsGrid.size() && !m_obstacle)
        {
            if (calculDistance(odom.position, m_pointsGrid[i]) < distObstacle) // Obstacle proche
            {
                //ROS_INFO("i : %d", i);
                //ROS_INFO("Obstacle proche");
                while ((i < m_pointsGrid.size()) && (calculDistance(odom.position, m_pointsGrid[i]) < distObstacle))
                {
                    m_vectorObstacle.push_back(m_pointsGrid[i]);
                    i++;
                }
                //int size = m_vectorObstacle.size();
                //ROS_INFO("Taille vector : %d", size);
                m_lengthObstacle = calculDistance(m_vectorObstacle[0], m_vectorObstacle[i-1]);
                pointTmp = m_vectorObstacle[i-1];
            
                float a = (pointArrivee.y - odom.position.y)/(pointArrivee.x - odom.position.x);
                float b = pointArrivee.y - a * pointArrivee.x;
                int j = 0;
                while (j < m_vectorObstacle.size() && !m_obstacle)
                {
                    //ROS_INFO("j : %d", j);
                    if (m_vectorObstacle[j].y == a * m_vectorObstacle[j].x + b)
                    {
                        //ROS_INFO("Obstacle détecté");
                        //ROS_INFO("Point obstacle : x = %f, y = %f", m_vectorObstacle[j].x, m_vectorObstacle[j].y);
                        m_obstacle = true;
                        m_obstacleLeft = m_vectorObstacle[0];
                        m_obstacleRight = pointTmp;
                    }
                    else
                    {
                        j++;
                    }
                }
                if (j == m_vectorObstacle.size())
                {
                    m_vectorObstacle.clear();
                }
            }
            else
            {
                i++;
            }
        }
    }
}*/

void DataMapObstacle::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    m_grid = grid;
    m_receiveGrid = true;
}
