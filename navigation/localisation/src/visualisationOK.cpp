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

std::vector<geometry_msgs::Point> tabLandmarks;
ros::Time g_landmarks_stamp;
std::vector<geometry_msgs::Point> tabSegments;
std::vector<geometry_msgs::Point> tabSegmentsVus;
ros::Time g_segments_stamp;
std::vector<geometry_msgs::Point> trajectoire;
std::vector<geometry_msgs::Point> odometrie;
geometry_msgs::Pose2D r;

void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments){
  tabSegments.clear();
  g_segments_stamp = segments->header.stamp;
  for (auto &it : segments->landmarks)
  {
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;

    tabSegments.push_back(p);
  }
}

void landmarksCallback(const deplacement_msg::LandmarksConstPtr& landmarks){
  tabLandmarks.clear();
  g_landmarks_stamp = landmarks->header.stamp;
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

void odomCallback(const nav_msgs::Odometry& new_odom){
  geometry_msgs::Point p;
  p.x = new_odom.pose.pose.position.x;
  p.y = new_odom.pose.pose.position.y;
  odometrie.push_back(p);
}

/*
void robotCallback(const geometry_msgs::Point& pos){
  r.x = pos.x;
  r.y = pos.y;
  trajectoire.push_back(pos);
}
*/

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");

  visualization_msgs::Marker   segments;
  visualization_msgs::Marker   landmarks;
  visualization_msgs::Marker   odom_brute;
  //visualization_msgs::Marker   robot;

  ros::NodeHandle n;

  ros::Subscriber sub_machines    = n.subscribe("objectDetection/landmarks", 1000, landmarksCallback);
  ros::Subscriber sub_segments    = n.subscribe("objectDetection/segments_global", 100000, segmentsCallback);
  ros::Subscriber sub_odom        = n.subscribe("objectDetection/new_odom", 1000, odomCallback);
  //ros::Subscriber sub_pos_robot   = n.subscribe("objectDetection/robot", 1000, robotCallback);

  ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rviz/visualization_markers", 1000000);

  ros::Rate rate(20);

  while (ros::ok())
  {
    segments.header.frame_id = "map";
    segments.header.stamp = g_segments_stamp;
    segments.ns = "visualisation_segments";
    segments.action = visualization_msgs::Marker::ADD;
    segments.pose.orientation.w = 1.0;
    segments.id = 2;
    segments.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    segments.scale.x = 0.1;
    // Line list is red
    segments.color.r = 1.0;
    segments.color.a = 1.0;
    segments.points = tabSegments;

    landmarks.header.frame_id = "robotino1/hardware/odom";
    landmarks.header.stamp = g_landmarks_stamp;
    landmarks.ns = "visualisation_machines";
    landmarks.action = visualization_msgs::Marker::ADD;
    landmarks.pose.orientation.w = 1.0;
    landmarks.id = 0;
    landmarks.type = visualization_msgs::Marker::LINE_LIST;
    //landmarks.type = visualization_msgs::Marker::POINTS;

    //largeur du landmark
    landmarks.scale.x = 0.35;

    //les landmarks sont verts
    landmarks.color.g = 1.0f;
    landmarks.color.a = 1.0;

    landmarks.points = tabLandmarks;

    odom_brute.header.frame_id = "map";
    odom_brute.header.stamp = ros::Time::now();
    odom_brute.ns = "visualisation_odom";
    odom_brute.action = visualization_msgs::Marker::ADD;
    odom_brute.pose.orientation.w = 1.0;
    odom_brute.id = 32;
    odom_brute.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    odom_brute.scale.x = 0.1;
    odom_brute.scale.y = 0.1;

    // o_brutdom is blue
    odom_brute.color.b = 1.0f;
    odom_brute.color.a = 1.0;

    odom_brute.points = odometrie;

    /*
    robot.header.frame_id = "/laser_link";
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
*/

    markers_pub.publish(segments);
    markers_pub.publish(landmarks);
    markers_pub.publish(odom_brute);
    //markers_pub.publish(robot);

    ros::spinOnce();

    rate.sleep();
  }
}
