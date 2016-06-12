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
#include "nav_msgs/GetMap.h"
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "occupancy_grid_utils/occupancy_grid_utils.h"
#include "occupancy_grid_utils/shape/LineSegment.h"
#include "occupancy_grid_utils/shape/ComposedShape.h"
#include "occupancy_grid_utils/shape/Rectangle.h"
#include "occupancy_grid_utils/grid_modifier/BasicGradientModifier.h"
#include "occupancy_grid_utils/grid_modifier/FastGradientModifier.h"

#include "common_utils/Parameter.h"
#include "geometry_utils/geometry_utils.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"

#include <set>
#include <string>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <memory>

void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines);
bool getMap_service(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

std::shared_ptr<occupancy_grid_utils::Shape> g_machinesShape(nullptr);
std::shared_ptr<occupancy_grid_utils::Shape> g_machinesShapeNoMargin(nullptr);
double g_margin = 0.3;
geometry_msgs::Point g_machineSize;
const float g_sizeX = 0.7;
const float g_sizeY = 0.35;
nav_msgs::OccupancyGrid mapLocalisation;
nav_msgs::OccupancyGrid mapLocalisationWithMachine;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_gen");
    ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("objectDetection/grid", 1);
    ros::Publisher mapFieldOnly_pub = nh.advertise<nav_msgs::OccupancyGrid>("objectDetection/grid_fieldOnly", 1);
    ros::Publisher mapFieldAndMachines_pub = nh.advertise<nav_msgs::OccupancyGrid>("objectDetection/grid_fieldAndMachines", 1);
    ros::ServiceServer getMap_server = nh.advertiseService("static_map", getMap_service);

    ros::Subscriber sub_poses_machine = nh.subscribe("objectDetection/landmarks", 10, Poses_Machine_Callback);

    ros::Rate loop_rate(1);

    //TODO paramêtrer la taille des machines
    g_machineSize.x = g_sizeX;
    g_machineSize.y = g_sizeY;

    //obtenir la configuration
    std::string fieldName;
    nh.param<std::string>("field/name",fieldName, "robocup_official");
    ROS_INFO_STREAM("Creating Map : " << fieldName);

    geometry_msgs::Point size;
    if (common_utils::getParameter(nh, "field/size", size) != 0)
    {
        std::cout << "size = " << size.x << ", " << size.y << std::endl;

        ROS_ERROR("Loading Map parameters : size : Wrong Size");
        return -1;
    }

    geometry_msgs::Point origin;
    if (common_utils::getParameter(nh, "field/origin", origin) != 0)
    {
        ROS_ERROR("Loading Map parameters : origin : Wrong Size");
        return -1;
    }

    double resolution = 0.05;
    nh.param<double>("field/resolution", resolution, 0.05);

    std::string frame_id = "map";
    nh.param<std::string>("field/frame_id", frame_id, "map");

    nh.param<double>("field/margin", g_margin, 0.3);

    //obtenir les murs
    std::list<occupancy_grid_utils::LineSegment> walls;
    if (common_utils::getParameter(nh, "field/walls", walls) !=0)
    {
        ROS_ERROR("Error loading field/walls");
        return -1;
    }

    //conversion des murs en rectangles
    std::list<occupancy_grid_utils::Rectangle> wallsRect;
    for (auto wall : walls)
    {
        geometry_msgs::Point p1,p2, mp;
        wall.getPoints(p1, p2);
        std::cout << wall.getName() << " : (" << p1.x << ", " << p1.y << "), (" << p2.x << ", " << p2.y << ")" << std::endl;

        mp = geometry_utils::midPoint(p1, p2);
        geometry_msgs::Pose2D pose;
        pose.x = mp.x;
        pose.y = mp.y;
        pose.theta = geometry_utils::angle(p1, p2);

        geometry_msgs::Point size;
        size.x = geometry_utils::distance(p1, p2);
        size.y = 0.05;

        occupancy_grid_utils::Rectangle wallRect(pose, size, g_margin);
        wallRect.setName(wall.getName());
        wallsRect.push_back(wallRect);

    }

    //charger zones interdites
    std::list<occupancy_grid_utils::Rectangle> forbidZones;
    if (common_utils::getParameter(nh, "field/forbidden_zones", forbidZones) !=0)
    {
        ROS_ERROR("Error loading field/forbidden_zones");
        return -1;
    }

    //definir les modifieurs
    std::shared_ptr<occupancy_grid_utils::GridModifier> modifiers(nullptr);

    //dégradés
    double distance = 0.5;
    nh.param<double>("field/gradient/distance", distance, 0.5);
    int minValue = 0;
    nh.param<int>("field/gradient/minValue", minValue, 0);
    std::shared_ptr<occupancy_grid_utils::FastGradientModifier> gradientModifier(new occupancy_grid_utils::FastGradientModifier(distance, minValue));

    modifiers = gradientModifier;

    // Empty map
    nav_msgs::OccupancyGrid map;
    occupancy_grid_utils::createEmptyMap(map, size, origin, frame_id, resolution);
    // Map for localisation
    occupancy_grid_utils::createEmptyMap(mapLocalisation, size, origin, frame_id, resolution);

    //draw map
    for (auto wall : wallsRect)
    {
        wall.draw(map);
    }
    for (auto forbidZone : forbidZones)
    {
        forbidZone.draw(map, 75);
    }
    // ROS Loop
    while(ros::ok())
    {
        //empty map
    	occupancy_grid_utils::createEmptyMap(mapLocalisation, size, origin, frame_id, resolution);
        //draw map
        for (auto wall : walls)
        {
            wall.draw(mapLocalisation);
        }
        for (auto forbidZone : forbidZones)
        {
            forbidZone.draw(mapLocalisation, 75);
        }
        mapFieldOnly_pub.publish(mapLocalisation);
        mapLocalisationWithMachine = mapLocalisation;
        if (g_machinesShape != nullptr)
        {
            g_machinesShapeNoMargin->draw(mapLocalisationWithMachine);
        }
        mapFieldAndMachines_pub.publish(mapLocalisationWithMachine);

        //empty map
    	occupancy_grid_utils::createEmptyMap(map, size, origin, frame_id, resolution);
        //draw map
        for (auto wall : wallsRect)
        {
            wall.draw(map);
        }
        for (auto forbidZone : forbidZones)
        {
            forbidZone.draw(map, 75);
        }
        if (g_machinesShape != nullptr)
        {
            g_machinesShape->draw(map);
        }
        if (modifiers != nullptr)
        {
            modifiers->execute(map);
        }


	 	map_pub.publish(map);
	 	ros::spinOnce();
		loop_rate.sleep();
 	}

    return 0;
}


void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines)
{
    std::shared_ptr<occupancy_grid_utils::ComposedShape> pShape(new occupancy_grid_utils::ComposedShape);
    std::shared_ptr<occupancy_grid_utils::ComposedShape> pShapeNoMargin(new occupancy_grid_utils::ComposedShape);

	for (int i=0; i< machines->landmarks.size(); i++)
    {
        std::shared_ptr<occupancy_grid_utils::Shape> rectangle(new occupancy_grid_utils::Rectangle(machines->landmarks[i], g_machineSize, g_margin));
        pShape->add(rectangle);
        std::shared_ptr<occupancy_grid_utils::Shape> rectangleNoMargin(new occupancy_grid_utils::Rectangle(machines->landmarks[i], g_machineSize, 0));
        pShapeNoMargin->add(rectangleNoMargin);
	}
    g_machinesShape = pShape;
    g_machinesShapeNoMargin = pShapeNoMargin;
}

bool getMap_service(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
    res.map = mapLocalisationWithMachine;
    return true;
}
