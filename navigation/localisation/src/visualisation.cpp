#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "deplacement_msg/Landmarks.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "cartographie_utils.h"

#include <cmath>

std::vector<geometry_msgs::Point> tabMachines;
ros::Time g_machines_stamp;
std::vector<geometry_msgs::Point> tabLandmarks;
ros::Time g_landmarks_stamp;
std::vector<geometry_msgs::Point> tabSegments;
ros::Time g_segments_stamp;
std::vector<geometry_msgs::Point> tabDroites;
std::vector<geometry_msgs::Point> trajectoire;
std::vector<geometry_msgs::Point> scan_global;
std::vector<geometry_msgs::Point> odometrie;
geometry_msgs::Pose2D r;

void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments)
{
  g_segments_stamp = segments->header.stamp;
  tabSegments.clear();
  for (auto &it : segments->landmarks)
  {
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;

    tabSegments.push_back(p);
  }
}

void droitesCallback(const deplacement_msg::LandmarksConstPtr& droites)
{
  tabDroites.clear();
  for (auto &it : droites->landmarks)
  {
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;

    tabDroites.push_back(p);
  }
}

void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{
  g_machines_stamp = machines->header.stamp;
  tabMachines.clear();
  for (auto &it : machines->landmarks)
  {

    //std::cout << "machine (" << it.x << "," << it.y << "," << it.theta << ")" << std::endl;
    //visualization_msgs::Marker m;
    //m.header.frame_id = "/odom";
    //m.header.stamp = ros::Time::now();
    //m.ns = "visualisation_machines";
    //m.action = visualization_msgs::Marker::ADD;
    //m.pose.position.x = it.x;
    //m.pose.position.y = it.y;
    //m.pose.orientation = tf::createQuaternionMsgFromYaw(it.theta);
    //m.id = getZone(it);
    //m.type = visualization_msgs::Marker::POINTS;

    //m.lifetime = 0;
  
    // POINTS markers use x and y scale for width/height respectively
    //m.scale.x = 0.35;
    //m.scale.y = 0.70;
  
      // Points are red
    //m.color.r = 1.0;
    //m.color.a = 1.0;

    //std::cout << "Marker : " << m << std::endl;


    geometry_msgs::Point pointA;
    pointA.x = it.x + cos(it.theta)*0.35;
    pointA.y = it.y + sin(it.theta)*0.35;
    tabMachines.push_back(pointA);

    geometry_msgs::Point pointB;
    pointB.x = it.x - cos(it.theta)*0.35;
    pointB.y = it.y - sin(it.theta)*0.35;
    tabMachines.push_back(pointB);
  }

  //std::cout << "MarkerArray : " << tabMachines << std::endl;
}

void landmarksCallback(const deplacement_msg::LandmarksConstPtr& landmarks)
{
  g_landmarks_stamp = landmarks->header.stamp;
  tabLandmarks.clear();
  for (auto &it : landmarks->landmarks)
  {
    geometry_msgs::Point pointA;
    pointA.x = it.x + cos(it.theta)*0.35;
    pointA.y = it.y + sin(it.theta)*0.35;
    tabLandmarks.push_back(pointA);

    geometry_msgs::Point pointB;
    pointB.x = it.x - cos(it.theta)*0.35;
    pointB.y = it.y - sin(it.theta)*0.35;
    tabLandmarks.push_back(pointB);
  }
}

void robotCallback(const geometry_msgs::Point& pos)
{
  r.x = pos.x;
  r.y = pos.y;
  trajectoire.push_back(pos);
}

void laserCallback(const deplacement_msg::LandmarksConstPtr& laser)
{
  scan_global.clear();
  for (auto &it : laser->landmarks)
  {
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;
    scan_global.push_back(p);
  }
} 

void odomCallback(const nav_msgs::Odometry& odom)
{
  geometry_msgs::Point p;
  p.x = odom.pose.pose.position.x;
  p.y = odom.pose.pose.position.y;
  odometrie.push_back(p);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");
  ROS_INFO_STREAM("Starting node visualisation");

  visualization_msgs::Marker   segments;
  visualization_msgs::Marker   droites;
  visualization_msgs::Marker   points;
  visualization_msgs::Marker   robot;
  visualization_msgs::Marker   laser;
  visualization_msgs::Marker   odom_brut;
  visualization_msgs::Marker   machines;
  visualization_msgs::Marker   landmarks;

  ros::NodeHandle n;

  ros::Subscriber sub_machines    = n.subscribe("objectDetection/machines", 1000, machinesCallback);
  ros::Subscriber sub_landmarks   = n.subscribe("objectDetection/landmarks", 1000, landmarksCallback);
  ros::Subscriber sub_segments    = n.subscribe("objectDetection/segments", 1000, segmentsCallback);
  ros::Subscriber sub_droites     = n.subscribe("objectDetection/droites", 1000, droitesCallback);
  ros::Subscriber sub_pos_robot   = n.subscribe("objectDetection/robot", 1000, robotCallback);
  ros::Subscriber sub_scan_global = n.subscribe("objectDetection/scan_global", 1000, laserCallback);
  ros::Subscriber sub_odom        = n.subscribe("objectDetection/new_odom", 1000, odomCallback);

  //ros::Publisher machines_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_machines", 10000);
  ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rviz/visualization_markers", 10000);

  ros::Rate rate(25);

  while (ros::ok())
  {
    // Segments
    segments.header.frame_id = "/laser_link";
    segments.header.stamp = g_segments_stamp;
    segments.ns = "visualisation_segments";
    segments.action = visualization_msgs::Marker::ADD;
    segments.pose.orientation.w = 1.0;
    segments.id = 2;
    segments.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    segments.scale.x = 0.05;

    // Line list is red
    segments.color.b = 1.0;
    segments.color.a = 0.7;

    segments.points = tabSegments;


    // Droites
    droites.header.frame_id = "/laser_link";
    droites.header.stamp = ros::Time::now();
    droites.ns = "visualisation_droites";
    droites.action = visualization_msgs::Marker::ADD;
    droites.pose.orientation.w = 1.0;
    droites.id = 2;
    droites.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    droites.scale.x = 0.1;

    // Line list is red
    droites.color.b = 1.0;
    droites.color.a = 1.0;

    droites.points = tabDroites;



    // Trajectoire
    points.header.frame_id = "/odom";
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
    
    // Machines
    machines.header.frame_id = "/laser_link";
    machines.header.stamp = g_machines_stamp;
    machines.ns = "visualisation_machines";
    machines.action = visualization_msgs::Marker::ADD;
    machines.pose.orientation.w = 1.0;
    machines.id = 0;
    machines.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    machines.scale.x = 0.35;

    // Machines are rose
    machines.color.r = 1.0f;
    machines.color.g = 0.8f;
    machines.color.b = 0.8f;
    machines.color.a = 0.7;

    machines.points = tabMachines;    

    // Landmarks
    landmarks.header.frame_id = "/odom";
    landmarks.header.stamp = g_landmarks_stamp;
    landmarks.ns = "visualisation_landmarks";
    landmarks.action = visualization_msgs::Marker::ADD;
    landmarks.pose.orientation.w = 1.0;
    landmarks.id = 0;
    landmarks.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    landmarks.scale.x = 0.35;

    // Landmarks are green
    landmarks.color.r = 0.0f;
    landmarks.color.g = 1.0f;
    landmarks.color.b = 0.0f;
    landmarks.color.a = 0.7;

    landmarks.points = tabLandmarks;


    // Laser
    laser.header.frame_id = "/odom";
    laser.header.stamp = ros::Time::now();
    laser.ns = "visualisation_laser";
    laser.action = visualization_msgs::Marker::ADD;
    laser.pose.orientation.w = 1.0;
    laser.id = 30;
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


    // Robot
    robot.header.frame_id = "/odom";
    robot.header.stamp = ros::Time::now();
    robot.ns = "visualisation_robot";
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.orientation.w = 1.0;
    robot.id = 31;
    robot.type = visualization_msgs::Marker::CYLINDER;

    // POINTS markers use x and y scale for width/height respectively
    robot.scale.x = 0.35;
    robot.scale.y = 0.35;
    robot.scale.z = 0.80;

    robot.pose.position.x = r.x;
    robot.pose.position.y = r.y;

    // Robot is blue
    robot.color.b = 1.0f;
    robot.color.a = 1.0;


    // Odométrie brute
    odom_brut.header.frame_id = "/odom";
    odom_brut.header.stamp = ros::Time::now();
    odom_brut.ns = "visualisation_odom";
    odom_brut.action = visualization_msgs::Marker::ADD;
    odom_brut.pose.orientation.w = 1.0;
    odom_brut.id = 32;
    odom_brut.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    odom_brut.scale.x = 0.1;
    odom_brut.scale.y = 0.1;

    // o_brutdom is blue
    odom_brut.color.r = 1.0f;
    //odom.color.b = 1.0f;
    odom_brut.color.a = 1.0;

    odom_brut.points = odometrie;


    // Publish markers
    markers_pub.publish(segments);
    markers_pub.publish(droites);
    markers_pub.publish(points);
    markers_pub.publish(robot);
    markers_pub.publish(laser);
    markers_pub.publish(odom_brut);
    markers_pub.publish(machines);
    markers_pub.publish(landmarks);

    //machines_pub.publish(tabMachines);

    ros::spinOnce();
    rate.sleep();
  }
}