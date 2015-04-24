#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include <cmath>

std::vector<geometry_msgs::Point> tabMachines;
std::vector<geometry_msgs::Point> tabSegments;
std::vector<geometry_msgs::Point> trajectoire;
std::vector<geometry_msgs::Point> scan_global;
std::vector<geometry_msgs::Point> odometrie;
geometry_msgs::Point r;

void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments){
  tabSegments.clear();
  for (auto &it : segments->landmarks){
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;
    /*std::cout << it.x << " | " << it.y << std::endl;*/
    tabSegments.push_back(p);
  }
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  tabMachines.clear();
  for (auto &it : machines->landmarks){
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;
    tabMachines.push_back(p);
  }
}

void robotCallback(const geometry_msgs::Point& pos){
  r.x = pos.x;
  r.y = pos.y;
  trajectoire.push_back(pos);
}

void laserCallback(const deplacement_msg::LandmarksConstPtr& laser){
  scan_global.clear();
  for (auto &it : laser->landmarks){
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;
    scan_global.push_back(p);
  }
} 

void odomCallback(const nav_msgs::Odometry& odom){
  geometry_msgs::Point p;
  p.x = odom.pose.pose.position.x;
  p.y = odom.pose.pose.position.y;
  odometrie.push_back(p);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");

  visualization_msgs::Marker line_list;
  visualization_msgs::Marker points;
  visualization_msgs::Marker robot;
  visualization_msgs::Marker machines;
  visualization_msgs::Marker laser;
  visualization_msgs::Marker odom_brut;

  ros::NodeHandle n;

  ros::Subscriber sub_machines    = n.subscribe("/landmarks", 1000, machinesCallback);
  ros::Subscriber sub_segments    = n.subscribe("/segments", 1000, segmentsCallback);
  ros::Subscriber sub_pos_robot   = n.subscribe("/robot", 1000, robotCallback);
  ros::Subscriber sub_scan_global = n.subscribe("/scan_global", 1000, laserCallback);
  ros::Subscriber sub_odom        = n.subscribe("/new_odom", 1000, odomCallback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10000);

  ros::Rate rate(20);

  while (ros::ok())
  {
    
    line_list.header.frame_id = "/laser_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "visualisation_segments";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.1;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    line_list.points = tabSegments;

    points.header.frame_id = "/laser_link";
    points.header.stamp = ros::Time::now();
    points.ns = "visualisation_trajectoire";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points.points = trajectoire;

    laser.header.frame_id = "/laser_link";
    laser.header.stamp = ros::Time::now();
    laser.ns = "visualisation_laser";
    laser.action = visualization_msgs::Marker::ADD;
    laser.pose.orientation.w = 1.0;
    laser.id = 3;
    laser.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    laser.scale.x = 0.1;
    laser.scale.y = 0.1;

    // Points are I don't know
    //laser.color.r = 1.0f;
    laser.color.b = 1.0f;
    laser.color.a = 1.0;

    //points.points = tabMachines;
    laser.points = scan_global;

    machines.header.frame_id = "/laser_link";
    machines.header.stamp = ros::Time::now();
    machines.ns = "visualisation_machines";
    machines.action = visualization_msgs::Marker::ADD;
    machines.pose.orientation.w = 1.0;
    machines.id = 0;
    machines.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    machines.scale.x = 0.35;
    machines.scale.y = 0.35;

    // Points are green
    machines.color.r = 1.0f;
    machines.color.a = 1.0;

    machines.points = tabMachines;

    robot.header.frame_id = "/laser_link";
    robot.header.stamp = ros::Time::now();
    robot.ns = "visualisation_robot";
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.orientation.w = 1.0;
    robot.id = 1;
    robot.type = visualization_msgs::Marker::CYLINDER;

    // POINTS markers use x and y scale for width/height respectively
    robot.scale.x = 0.35;
    robot.scale.y = 0.35;
    robot.scale.z = 0.80;

    robot.pose.position.x = r.x;
    robot.pose.position.y = r.y;
    robot.pose.orientation.x = 0.0;
    robot.pose.orientation.y = 0.0;
    robot.pose.orientation.z = 0.0;
    robot.pose.orientation.w = 1.0;

    // Robot is blue
    robot.color.b = 1.0f;
    robot.color.a = 1.0;

    odom_brut.header.frame_id = "/laser_link";
    odom_brut.header.stamp = ros::Time::now();
    odom_brut.ns = "visualisation_odom";
    odom_brut.action = visualization_msgs::Marker::ADD;
    odom_brut.pose.orientation.w = 1.0;
    odom_brut.id = 7;
    odom_brut.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    odom_brut.scale.x = 0.1;
    odom_brut.scale.y = 0.1;

    // o_brutdom is blue
    odom_brut.color.r = 1.0f;
    //odom.color.b = 1.0f;
    odom_brut.color.a = 1.0;

    odom_brut.points = odometrie;

    //marker_pub.publish(line_list);
    marker_pub.publish(points);
    marker_pub.publish(robot);
    marker_pub.publish(machines);
    marker_pub.publish(laser);
    marker_pub.publish(odom_brut);
    marker_pub.publish(line_list);

    ros::spinOnce();

    rate.sleep();
  }
}