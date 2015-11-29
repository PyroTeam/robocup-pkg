#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

double g_x, g_y, g_z;
ros::Publisher g_pubOdom;

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
    ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel", 1, &cmdVelCallback);
    g_pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
    gazebo::transport::SubscriberPtr subGps = node->Subscribe("/gazebo/pyro_2015/robotino_pyro/gazsim/gps/", &gpsCallback);

    // Publisher loop...replace with your own code.
    g_x=0; g_y=0; g_z=0; 
    gazebo::math::Vector3 vect(g_x,g_y,g_z);
    while (ros::ok())
    {
        // Throttle Publication
        gazebo::common::Time::MSleep(100);

        // Generate a pose
        vect.Set(g_x, g_y, g_z);

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
    g_x=msg->linear.x; 
    g_y=msg->linear.y; 
    g_z=msg->angular.z;
}

void gpsCallback(ConstPosePtr &msg)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id="/base_link";
    odom_msg.header.frame_id="/odom";
    odom_msg.header.stamp=ros::Time::now();

    odom_msg.pose.pose.position.x=msg->position().x();
    odom_msg.pose.pose.position.y=msg->position().y();
    odom_msg.pose.pose.position.z=0;

    odom_msg.pose.pose.orientation.x=msg->orientation().x();
    odom_msg.pose.pose.orientation.y=msg->orientation().y();
    odom_msg.pose.pose.orientation.z=msg->orientation().z();
    odom_msg.pose.pose.orientation.w=msg->orientation().w();

    odom_msg.twist.twist.linear.x=g_x;
    odom_msg.twist.twist.linear.y=g_y;
    odom_msg.twist.twist.linear.z=0;
    odom_msg.twist.twist.angular.x=0;
    odom_msg.twist.twist.angular.y=0;
    odom_msg.twist.twist.angular.z=g_z;

    g_pubOdom.publish(odom_msg);
}