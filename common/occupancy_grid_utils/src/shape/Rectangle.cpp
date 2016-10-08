/**
 * \file        Rectangle.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "shape/Rectangle.h"

namespace occupancy_grid_utils {

Rectangle::Rectangle() : Shape(), m_x(0.0), m_y(0.0), m_theta(0.0), m_height(0.0),
                         m_width(0.0), m_margin(0.0)
{

}

Rectangle::Rectangle(const geometry_msgs::Pose2D &pose, const geometry_msgs::Point &size, const float margin) : Shape(),
    m_x(pose.x),
    m_y(pose.y),
    m_theta(pose.theta),
    m_width(size.x),
    m_height(size.y),
    m_margin(margin)
{

}

Rectangle::~Rectangle()
{

}

void Rectangle::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{
    //Code provenant de l'ancien GridMaker
    
    // Environment
    	// static bool alreadyDone = false;
    	// if(alreadyDone && 0)
    	// 	return -1;
    	// alreadyDone = true;

    	float res = grid.info.resolution;
    	const int samplingMultiplier = 2;
    	float totalDrawHeight = (2*m_margin+m_height);
    	float totalDrawWidth = (2*m_margin+m_width);


    	// ROS_INFO("totalDrawHeight %f", totalDrawHeight);
    	// ROS_INFO("totalDrawWidth %f", totalDrawWidth);
    	// ROS_INFO("__________________________________");

    // Algo
    	for (int i = 0; i < (int)(totalDrawWidth*samplingMultiplier/res); ++i)
    	{
    		for (int j = 0; j < (int)(totalDrawHeight*samplingMultiplier/res); ++j)
    		{
    			// ROS_INFO("%d:%d",i,j);
    			// Define the gain for this point
    			int gain = 0;
    			// Outside of the rect (margin zones)
    			if(	i < (int)(m_margin*samplingMultiplier/res)
    				|| i > (int)((m_margin+m_width)*samplingMultiplier/res)
    				|| j < (int)(m_margin*samplingMultiplier/res)
    				|| j > (int)((m_margin+m_height)*samplingMultiplier/res))
    			{
    				// Variable gain, proportional to the distance with the rect
    				// 	/!\ Temporarily to 50
    				gain = max_value;
    			}
    			// Into the rect
    			else
                {
    				// Static gain, set to maximum (100)
    				gain = max_value;
    			}

    			// ROS_INFO("Gain : %d",gain);

    			// Get the coords
    			float deltaX = -totalDrawWidth/2.0 + i*res/samplingMultiplier;
    			float deltaY = totalDrawHeight/2.0 - j*res/samplingMultiplier;

    			// ROS_INFO("deltaX : %f",deltaX);
    			// ROS_INFO("deltaY : %f",deltaY);

    			// ROS_INFO("dXc %f dYs %f",deltaX*cos(theta),deltaY*sin(theta));
    			// ROS_INFO("dYc %f dXs %f",deltaY*cos(theta),deltaX*sin(theta));
    			// float xP = x + deltaX*cos(theta) - deltaY*sin(theta);
    			// float yP = y - deltaY*cos(theta) - deltaX*sin(theta);
    			float xP = m_x + deltaX*cos(m_theta) + deltaY*sin(m_theta);
    			float yP = m_y - deltaY*cos(m_theta) + deltaX*sin(m_theta);

    			// ROS_INFO("xP : %f",xP);
    			// ROS_INFO("yP : %f",yP);

                int cellValue = getCellValue(grid, xP, yP);
    			// ROS_INFO("cell : %d",cell);

    			// Fill it
                setCell(grid, xP, yP, std::max(gain, cellValue));
    		}

    		// ROS_INFO("\t__________________________________");
    	}

    	// return 0;
}

} // namespace occupancy_grid_utils
