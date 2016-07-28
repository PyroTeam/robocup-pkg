#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Machines.h"
#include "deplacement_msg/Robot.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include <sensor_msgs/PointCloud.h>

#include <cmath>

std::vector<geometry_msgs::Point> tabLandmarks;
std::vector<geometry_msgs::Point> tabSegments;
deplacement_msg::Robot g_robot;
std_msgs::Header g_l_header;
std_msgs::Header g_s_header;
//std_msgs::Header g_p_header;


void segmentsCallback(const deplacement_msg::LandmarksConstPtr& segments){
  tabSegments.clear();
  g_s_header = segments->header;
  for (auto &it : segments->landmarks)
  {
    geometry_msgs::Point p;
    p.x = it.x;
    p.y = it.y;

    tabSegments.push_back(p);
  }
}

void landmarksCallback(const deplacement_msg::MachinesConstPtr& landmarks)
{
  tabLandmarks.clear();
  g_l_header = landmarks->header;
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

void robotCallback(const deplacement_msg::Robot &p)
{
  g_robot = p;
}

/*
void pclCallback(const sensor_msgs::PointCloudConstPtr& pcl)
{
    tabPointCloud.clear();
    geometry_msgs::Point zero;
    zero.x = 0.0;
    zero.y = 0.0;
    zero.z = 0.0;
    g_p_header = pcl->header;
    int cpt = 1;
    for (int i = 0; i < pcl->points.size(); i=i+1)
    {
        tabPointCloud.push_back(zero);
        geometry_msgs::Point p;
        p.x = pcl->points[i].x;
        p.y = pcl->points[i].y;
        tabPointCloud.push_back(p);
        p.x = pcl->points[i+1].x;
        p.y = pcl->points[i+1].y;
        tabPointCloud.push_back(p);
    }
}
*/
int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualisation");

  visualization_msgs::Marker landmarks;
  visualization_msgs::Marker segments;
  visualization_msgs::Marker pcl;
  visualization_msgs::Marker robotMarker;

  ros::NodeHandle n;
  std::string tf_prefix;
  n.param<std::string>("simuRobotNamespace", tf_prefix, "");
  if (tf_prefix.size() != 0)
  {
      tf_prefix += "/";
  }

  ros::Subscriber sub_machines    = n.subscribe("objectDetection/landmarks", 1, landmarksCallback);
  ros::Subscriber sub_segments    = n.subscribe("objectDetection/segments", 1, segmentsCallback);
  ros::Subscriber sub_robot    = n.subscribe("objectDetection/robot", 1, robotCallback);
  //ros::Subscriber sub_laser       = n.subscribe("hardware/scan_pcl", 1, pclCallback);

  ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rviz/visualization_markers", 1000000);

  ros::Rate rate(30);

  while (ros::ok())
  {
    landmarks.header = g_l_header;
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

    segments.header = g_s_header;
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
/*
    pcl.header = g_p_header;
    pcl.ns = "visualisation_pcl";
    pcl.action = visualization_msgs::Marker::ADD;
    pcl.pose.orientation.w = 1.0;
    pcl.id = 3;
    pcl.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    pcl.scale.x = 1;
    pcl.scale.y = 1;
    pcl.scale.z = 1;
    pcl.color.b = 1.0;
    pcl.color.a = 1.0;
    pcl.points = tabPointCloud;*/

    robotMarker.header = g_robot.header;
    robotMarker.ns = "visualisation_robot";
    robotMarker.action = visualization_msgs::Marker::ADD;
    robotMarker.pose.orientation.w = 1.0;
    robotMarker.id = 5;
    robotMarker.type = visualization_msgs::Marker::ARROW;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    robotMarker.scale.x = 1;
    robotMarker.scale.y = 0.1;
    robotMarker.scale.z = 0.1;
    // Line list is red
    robotMarker.color.r = 1.0;
    robotMarker.color.a = 1.0;
    geometry_msgs::Pose tmp;
    tmp.position.x = g_robot.pose.x;
    tmp.position.y = g_robot.pose.y;
    tmp.orientation = tf::createQuaternionMsgFromYaw(g_robot.pose.theta);
    robotMarker.pose = tmp;

    markers_pub.publish(landmarks);
    markers_pub.publish(segments);
    //markers_pub.publish(pcl);
    markers_pub.publish(robotMarker);

    ros::spinOnce();
    rate.sleep();
  }
}
