#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Machines.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include <cmath>

std::vector<geometry_msgs::Point> tabLandmarks;
ros::Time g_landmarks_stamp;

void landmarksCallback(const deplacement_msg::MachinesConstPtr& landmarks){
  tabLandmarks.clear();
  g_landmarks_stamp = landmarks->header.stamp;
  for (auto &it : landmarks->landmarks)
  {
    geometry_msgs::Point pointA;
    pointA.x = it.pose.x + cos(it.pose.theta)*0.35;
    pointA.y = it.pose.y + sin(it.pose.theta)*0.35;
    tabLandmarks.push_back(pointA);

    geometry_msgs::Point pointB;
    pointB.x = it.pose.x - cos(it.pose.theta)*0.35;
    pointB.y = it.pose.y - sin(it.pose.theta)*0.35;
    tabLandmarks.push_back(pointB);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");

  visualization_msgs::Marker   landmarks;

  ros::NodeHandle n;
  std::string tf_prefix;
  n.param<std::string>("simuRobotNamespace", tf_prefix, "");
  if (tf_prefix.size() != 0)
  {
      tf_prefix += "/";
  }

  ros::Subscriber sub_machines    = n.subscribe("objectDetection/landmarks", 1000, landmarksCallback);

  ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rviz/visualization_markers", 1000000);

  ros::Rate rate(20);

  while (ros::ok())
  {
    landmarks.header.frame_id = "map";
    landmarks.header.stamp = g_landmarks_stamp;
    landmarks.ns = "visualisation_machines";
    landmarks.action = visualization_msgs::Marker::ADD;
    landmarks.pose.orientation.w = 1.0;
    landmarks.id = 0;
    landmarks.type = visualization_msgs::Marker::LINE_LIST;

    //largeur du landmark
    landmarks.scale.x = 0.35;

    //les landmarks sont verts
    landmarks.color.g = 1.0f;
    landmarks.color.a = 1.0;

    landmarks.points = tabLandmarks;

    markers_pub.publish(landmarks);

    ros::spinOnce();
    rate.sleep();
  }
}
