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
std::vector<geometry_msgs::Point> tabMachines;
std::vector<geometry_msgs::Point> tabSegments;
deplacement_msg::Robot g_robot;
std_msgs::Header g_l_header;
std_msgs::Header g_s_header;
std_msgs::Header g_m_header;


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


void machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{
    tabMachines.clear();
    g_m_header = machines->header;
    for (auto &it : machines->landmarks)
    {
        geometry_msgs::Point pointA;
        pointA.x = it.x + cos(it.theta)*0.35;
        pointA.y = it.y + sin(it.theta)*0.35;
        tabMachines.push_back(pointA);

        geometry_msgs::Point pointB;
        pointB.x = it.x - cos(it.theta)*0.35;
        pointB.y = it.y - sin(it.theta)*0.35;
        tabMachines.push_back(pointB);
    }
}

void robotCallback(const deplacement_msg::Robot &p)
{
    g_robot = p;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "visualisation");

    visualization_msgs::Marker landmarks;
    visualization_msgs::Marker machines;
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

    ros::Subscriber sub_landmarks = n.subscribe("objectDetection/landmarks", 1, landmarksCallback);
    ros::Subscriber sub_machines  = n.subscribe("objectDetection/machines", 1, machinesCallback);
    ros::Subscriber sub_segments  = n.subscribe("objectDetection/segments", 1, segmentsCallback);
    ros::Subscriber sub_robot     = n.subscribe("objectDetection/robot", 1, robotCallback);

    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rviz/visualization_markers", 1000000);

    ros::Rate rate(30);

    while (ros::ok())
    {
        landmarks.header = g_l_header;
        landmarks.ns = "visualisation_landmarks";
        landmarks.action = visualization_msgs::Marker::ADD;
        landmarks.pose.orientation.w = 1.0;
        landmarks.id = 0;
        landmarks.type = visualization_msgs::Marker::LINE_LIST;
        landmarks.scale.x = 0.35;
        landmarks.color.g = 1.0f;
        landmarks.color.a = 1.0;
        landmarks.points = tabLandmarks;

        machines.header = g_m_header;
        machines.ns = "visualisation_machines";
        machines.action = visualization_msgs::Marker::ADD;
        machines.pose.orientation.w = 1.0;
        machines.id = 0;
        machines.type = visualization_msgs::Marker::LINE_LIST;
        machines.scale.x = 0.35;
        machines.color.b = 1.0f;
        machines.color.a = 1.0;
        machines.points = tabMachines;

        segments.header = g_s_header;
        segments.ns = "visualisation_segments";
        segments.action = visualization_msgs::Marker::ADD;
        segments.pose.orientation.w = 1.0;
        segments.id = 2;
        segments.type = visualization_msgs::Marker::LINE_LIST;
        segments.scale.x = 0.1;
        segments.color.r = 1.0;
        segments.color.a = 1.0;
        segments.points = tabSegments;


        robotMarker.header = g_robot.header;
        robotMarker.ns = "visualisation_robot";
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.pose.orientation.w = 1.0;
        robotMarker.id = 5;
        robotMarker.type = visualization_msgs::Marker::ARROW;
        robotMarker.scale.x = 1;
        robotMarker.scale.y = 0.1;
        robotMarker.scale.z = 0.1;
        robotMarker.color.r = 1.0;
        robotMarker.color.a = 1.0;
        geometry_msgs::Pose tmp;
        tmp.position.x = g_robot.pose.x;
        tmp.position.y = g_robot.pose.y;
        tmp.orientation = tf::createQuaternionMsgFromYaw(g_robot.pose.theta);
        robotMarker.pose = tmp;

        markers_pub.publish(landmarks);
        markers_pub.publish(machines);
        markers_pub.publish(segments);
        markers_pub.publish(robotMarker);

        ros::spinOnce();
        rate.sleep();
    }
}
