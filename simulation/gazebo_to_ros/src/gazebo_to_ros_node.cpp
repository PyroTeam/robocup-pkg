
/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>

#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

double x, y, z; 

ros::Publisher pubOdom;

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
void gpsCallback(ConstPosePtr &msg);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Load gazebo
  gazebo::setupClient(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ros::init(argc, argv, "publisher");
  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("/gazebo/pyro_2015/robotino_pyro/RobotinoSim/MotorMove/");

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  // Subscriber
	ros::NodeHandle nh;
	// ros::Subscriber cmd_vel_sub_;
  	// cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &WorldPluginTutorial::cmdVelCallback, this);
  	ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel", 1, &cmdVelCallback);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
  gazebo::transport::SubscriberPtr subGps = node->Subscribe("/gazebo/pyro_2015/robotino_pyro/gazsim/gps/", &gpsCallback);

  // Publisher loop...replace with your own code.
x=0; y=0; z=0; 
    gazebo::math::Vector3 vect(x,y,z);
  while (ros::ok())
  {
    // Throttle Publication
    gazebo::common::Time::MSleep(100);

    // Generate a pose
    vect.Set(x, y, z);

    // Convert to a pose message
    gazebo::msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, vect);

    pub->Publish(msg);
    ros::spinOnce();
  }

  // Make sure to shut everything down.
  gazebo::shutdown();
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
x=msg->linear.y; 
y=-msg->linear.x; 
z=msg->angular.z; 
	// double linear_x = msg->linear.x;
	// double linear_y = msg->linear.y;
	// double angular = msg->angular.z;
}

void gpsCallback(ConstPosePtr &msg)
{

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x=msg->position().x();
  odom_msg.pose.pose.position.y=msg->position().y();
  odom_msg.pose.pose.position.z=0;

  odom_msg.pose.pose.orientation.x=msg->orientation().x();
  odom_msg.pose.pose.orientation.y=msg->orientation().y();
  odom_msg.pose.pose.orientation.z=msg->orientation().z();
  odom_msg.pose.pose.orientation.w=msg->orientation().w();

  pubOdom.publish(odom_msg);
}