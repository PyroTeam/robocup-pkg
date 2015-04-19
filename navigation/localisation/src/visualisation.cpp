#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"

#include <cmath>

std::vector<geometry_msgs::Point> tabMachines;
std::vector<geometry_msgs::Point> tabSegments;
geometry_msgs::Point r;

//cette putin de fonction de callback ne marche pas !!!
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
  geometry_msgs::Point p;
  r.x = pos.x;
  r.y = pos.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");

  visualization_msgs::Marker line_list;
  visualization_msgs::Marker points;
  visualization_msgs::Marker robot;

  ros::NodeHandle n;

  ros::Subscriber sub_machines  = n.subscribe("/machines", 1000, machinesCallback);
  ros::Subscriber sub_segments  = n.subscribe("/segments", 1000, segmentsCallback);
  ros::Subscriber sub_pos_robot = n.subscribe("/robot", 1000, robotCallback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

  ros::Rate rate(30);

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
    points.ns = "visualisation_machines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.35;
    points.scale.y = 0.7;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points.points = tabMachines;

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

    //marker_pub.publish(line_list);
    marker_pub.publish(points);
    marker_pub.publish(robot);

    ros::spinOnce();

    rate.sleep();
  }
}