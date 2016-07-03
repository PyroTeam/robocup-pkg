#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <robotino_msgs/ResetOdometry.h>
#include <geometry_utils/geometry_utils.h>
#include "LaserScan.h"

std::string g_color;
std::string g_ns;
tf::TransformListener *g_tf_listener;
bool g_alreadyDone;

robotino_msgs::ResetOdometry gimmePositionBiatch(laserScan scan)
{
  robotino_msgs::ResetOdometry setpoint;
  const float max_distance = 1.0;
  float distance = max_distance;

  // min1 correspond au entry[COLOR]2, min2 au entry[COLOR]1, max au coin
  geometry_msgs::Point min1, max, min2;
  int indexMin1 = 0, indexMin2 = 0, indexMax = 0;

  for (int i = 0; i < scan.getRanges().size()/2; ++i)
  {
    if (scan.getRanges()[i] <= distance && scan.getRanges()[i] < max_distance)
    {
      //ROS_ERROR("Min 1 RESEARCH >>> Range [%d] : %f", i, scan.getRanges()[i]);
      distance = scan.getRanges()[i];
      min1 = scan.getPointVector()[i];
      indexMin1 = i;
    }
  }

  distance = max_distance;

  if (scan.getRanges().size() > 1)
  {
    for (int j = scan.getRanges().size()-1; j > scan.getRanges().size()/2; j--)
    {
      if (scan.getRanges()[j] <= distance && scan.getRanges()[j] < max_distance)
      {
        distance = scan.getRanges()[j];
        min2 = scan.getPointVector()[j];
        indexMin2 = j;
        //ROS_ERROR("Min 2 RESEARCH >>> Range [%d] : %f", j, scan.getRanges()[j]);
      }
    }
  }

  int k;
  //ROS_INFO("Searching local maximum between %d and %d", indexMin1, indexMin2);
  for (k = indexMin1; k < indexMin2; k++)
  {
    //ROS_ERROR("Range [%d] : %f >? %f && < 1.0",k, scan.getRanges()[k], distance);
    if (scan.getRanges()[k] > distance && scan.getRanges()[k] < max_distance)
    {
      distance = scan.getRanges()[k];
      max = scan.getPointVector()[k];
      indexMax = k;
    }
  }

  ROS_WARN("Min 1 : %d", indexMin1);
  ROS_WARN("Min 2 : %d", indexMin2);
  ROS_WARN("Max   : %d", indexMax);

  // REMINDER: min1 correspond au entry[COLOR]2, min2 au entry[COLOR]1, max au coin
/*
______________________             ________________________
                                  |
                          OOO     |
                         OOOOO    | xMagentaEntry
                          OOO     |
__________________________________|
             yBottom
  */
  // const double yBottom = -1.12; // voir fieldRoboCup2016.yaml
  // const double xMagentaEntry = -2.735; // voir fieldRoboCup2016.yaml

  const double yBottom = -1; // voir field_simulator.yaml
  const double xMagentaEntry = -3; // voir field_simulator.yaml

  geometry_msgs::Point zero;
  zero.x = 0.0;
  zero.y = 0.0;

  double dx = geometry_utils::distance(zero,min1);
  double dy = geometry_utils::distance(zero,min2);

  ROS_INFO("dx = %f | dy := %f", dx, dy);

  tf::StampedTransform transform;
  try
  {
    g_tf_listener->waitForTransform(g_ns+"base_link",g_ns+"laser_link",scan.getTime(),ros::Duration(1.0));
    g_tf_listener->lookupTransform(g_ns+"base_link",g_ns+"laser_link",scan.getTime(), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s",ex.what());
  }

  geometry_msgs::Pose2D setpointPose2DLaserFrame;
  setpointPose2DLaserFrame.x = dx;
  setpointPose2DLaserFrame.y = dy;
  geometry_msgs::Pose2D setpointPose2DBaseFrame = geometry_utils::changeFrame(setpointPose2DLaserFrame, transform);

  setpoint.request.x = xMagentaEntry - setpointPose2DBaseFrame.x;
  setpoint.request.y = yBottom + setpointPose2DBaseFrame.y;
  setpoint.request.phi = atan2(-setpointPose2DBaseFrame.y,setpointPose2DBaseFrame.x);

  if (dx != 0.0 && dy != 0.0)
  {
    g_alreadyDone = true;
  }

  return setpoint;
}

int main(int argc, char** argv)
{
    //Initialisation du noeud ROS
    ros::init(argc, argv, "recalage");
    ROS_INFO("Starting node recalage_node");

    tf::TransformListener tf_listener;
    g_tf_listener = &tf_listener;
    g_alreadyDone = false;

    // Objet laser (convertit et contient les points laser en coord cartesiennes)
    laserScan laserData;

    ros::NodeHandle n;
    n.param<std::string>("robotNamespace", g_ns, "");
    n.param<std::string>("teamColor", g_color, "magenta");

    // Subscribe to laserScan
    ros::Subscriber sub_laser  = n.subscribe("hardware/scan", 1, &laserScan::laserCallback, &laserData);

    ros::ServiceClient client = n.serviceClient<robotino_msgs::ResetOdometry>("reset_odometry");

    // Initialisation du random
    srand(time(NULL));

    // Le noeud tourne à 10Hz
    ros::Rate loop_rate (10);
    while(n.ok() && !g_alreadyDone)
    {
      robotino_msgs::ResetOdometry resetPosition = gimmePositionBiatch(laserData);

      if (client.call(resetPosition))
      {
        ROS_INFO("Reset done with the position (%f,%f,%f)", resetPosition.request.x, resetPosition.request.y, resetPosition.request.phi);
      }
      else
      {
        ROS_ERROR("Failed to call service reset_odometry");
      }

      // Spin
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
