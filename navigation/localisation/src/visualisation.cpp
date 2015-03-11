#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points;
    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Create the vertices for the points and lines
    for (int32_t i = 0; i < 100; ++i)
    {
      float y = 5 * (i - 50)/100.0;
      float z = 0;

      geometry_msgs::Point p;
      p.x = y*y;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
    }


    marker_pub.publish(points);

    r.sleep();

    f += 0.04;
  }
}