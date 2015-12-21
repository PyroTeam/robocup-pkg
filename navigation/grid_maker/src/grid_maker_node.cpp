/**
 * \file        grid_maker_node.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "occupancy_grid_utils/occupancy_grid_utils.h"
#include "occupancy_grid_utils/shape/LineSegment.h"

#include <set>
#include <string>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <memory>

std::string execProcess(std::string cmd)
{
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        return "ERROR";
    }
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get()))
    {
        if (fgets(buffer, 128, pipe.get()) != NULL)
        {
            result += buffer;
        }
    }
    return result;
}

int loadPoint(ros::NodeHandle &nh, const std::string &paramName, geometry_msgs::Point &p)
{
    std::vector<double> param_vec;
    std::vector<double> defaultParam_vec;
    nh.param<std::vector<double>>(paramName, param_vec, defaultParam_vec);
    if (param_vec.size()<2)
    {
        ROS_ERROR("Loading Map parameters : Wrong Size");
        return -1;
    }
    p.x = param_vec[0];
    p.y = param_vec[1];
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_gen");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("objectDetection/grid", 1);

    ros::Rate loop_rate(1);


    //obtenir la configuration
    std::string fieldName;
    nh.param<std::string>("field/name",fieldName, "robocup");
    ROS_INFO_STREAM("Creating Map : " << fieldName);

    geometry_msgs::Point size;
    if (loadPoint(nh, "field/size", size) != 0)
    {
        std::cout << "size = " << size.x << ", " << size.y << std::endl;

        ROS_ERROR("Loading Map parameters : size : Wrong Size");
        return -1;
    }

    geometry_msgs::Point origin;
    if (loadPoint(nh, "field/origin", origin) != 0)
    {
        ROS_ERROR("Loading Map parameters : origin : Wrong Size");
        return -1;
    }

    double resolution = 0.05;
    nh.param<double>("field/resolution", resolution, 0.05);

    std::string frame_id = "map";
    nh.param<std::string>("field/frame_id", frame_id, "map");

    //obtenir les murs

    std::string command("rosparam list ");
    std::string rosNameSpace("");
    std::string wallParamName("/field/wall");
    std::string wallStr = execProcess(command + rosNameSpace + wallParamName);

    std::set<std::string> wallSet;
    std::stringstream sWallStr(wallStr);

    //+1 pour tenir compte du '/' de separation après 'field/wall'
    size_t wallParamNameSize = rosNameSpace.size() + wallParamName.size() + 1;

    while (!sWallStr.eof())
    {
        std::string wall;
        std::getline(sWallStr, wall, '\n');
        if (!sWallStr.eof() && wall.size() > wallParamNameSize)
        {
            std::size_t prefixFound = wall.find(rosNameSpace + wallParamName);
            prefixFound += wallParamNameSize;
            wall = wall.substr(prefixFound);
            std::size_t found = wall.find("/");
            if (found != std::string::npos)
            {
                wall = wall.substr(0, found);
                wallSet.insert(wall);
            }
        }
    }

    std::list<occupancy_grid_utils::LineSegment> walls;
    for (auto wall : wallSet)
    {
        //charge la config d'un mur
        geometry_msgs::Point start;
        if (loadPoint(nh, "field/wall/"+wall+"/start", start) != 0)
        {
            ROS_ERROR_STREAM("Loading Map parameters : " << wall << " start : Wrong Size");
            return -1;
        }

        geometry_msgs::Point end;
        if (loadPoint(nh, "field/wall/"+wall+"/end", end) !=0)
        {
            ROS_ERROR_STREAM("Loading Map parameters : " << wall << " end : Wrong Size");
            return -1;
        }
        occupancy_grid_utils::LineSegment wallSegment(start, end);
        wallSegment.setName(wall);
        walls.push_back(wallSegment);
    }

    for (auto wall : walls)
    {
        geometry_msgs::Point p1,p2;
        wall.getPoints(p1, p2);
        std::cout << wall.getName() << " : (" << p1.x << ", " << p1.y << "), (" << p2.x << ", " << p2.y << ")" << std::endl;

    }

    // Empty map
    nav_msgs::OccupancyGrid map;
    occupancy_grid_utils::createEmptyMap(map, size, origin, frame_id, resolution);
    //draw Wall
    for (auto wall : walls)
    {
        wall.draw(map);
    }

    // ROS Loop
    while(ros::ok())
    {
        //empty map
    	occupancy_grid_utils::createEmptyMap(map, size, origin, frame_id, resolution);
        //draw wall
        for (auto wall : walls)
        {
            wall.draw(map);
        }
        
	 	map_pub.publish(map);
	 	ros::spinOnce();
		loop_rate.sleep();
 	}

    return 0;
}
