/**
 * \file        simple_cloud_laser_plugin.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 *              Hage-Chehade Sandra
 * \date        2016-06-30
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */
#include <ros/ros.h>
#include <list>

#include "occupancy_grid_utils/shape/Rectangle.h"
#include "occupancy_grid_utils/shape/LineSegment.h"
#include "common_utils/parameterFunctions.h"
#include "geometry_msgs/Pose2D.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zoneSizeCalcul");

    ros::NodeHandle nh;

    //obtenir les murs
    std::list<occupancy_grid_utils::LineSegment> walls;
    if (common_utils::getParameter(nh, "field/walls", walls) !=0)
    {
        ROS_ERROR("Error loading field/walls");
        return -1;
    }

    float minX=0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;


    //conversion des murs en rectangles
    for (auto wall : walls)
    {
        geometry_msgs::Point p1,p2, mp;
        wall.getPoints(p1, p2);
        //std::cout << wall.getName() << " : (" << p1.x << ", " << p1.y << "), (" << p2.x << ", " << p2.y << ")" << std::endl;
        if (wall.getName() == "bottom1")
        {
            minY = p1.y;
        }
        if (wall.getName() == "top1")
        {
            maxY = p1.y;
        }
        if (wall.getName() == "left1")
        {
            minX = p1.x;
        }
        if (wall.getName() == "right1")
        {
            maxX = p1.x;
        }
    }

    float zone_width = (maxX-minX)/6.0;
    float zone_height = (maxY-minY)/4.0;

    nh.setParam("zone_width", zone_width);
    nh.setParam("zone_height", zone_height);


    return 0;
};
