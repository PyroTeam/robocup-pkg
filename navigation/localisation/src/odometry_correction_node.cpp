#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

nav_msgs::Odometry prec;

ros::Publisher pub_odom;

inline double getTimeAsDouble(ros::Time t){
    return double(t.sec) + double(t.nsec)/1000000000.0;
}

void odomCallback(const nav_msgs::Odometry& odom){
    nav_msgs::Odometry newOdom = odom;

    double diff = tf::getYaw(odom.pose.pose.orientation) - tf::getYaw(prec.pose.pose.orientation);
    if (diff > M_PI){
        diff -= 2*M_PI;
    }
    else if (diff < -M_PI){
        diff += 2*M_PI;
    }

    newOdom.twist.twist.angular.z =  diff / (getTimeAsDouble(odom.header.stamp) - getTimeAsDouble(prec.header.stamp));

    //on publie
    pub_odom.publish(newOdom);

    prec = odom;
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting node odometry_correction_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "odometry_correction_node");

    ros::NodeHandle n;

    //on va publier l'odométrie corrigée sur /new_odom
    pub_odom = n.advertise<nav_msgs::Odometry>("/new_odom", 1000);

    prec.pose.pose.orientation.x = 0.0;
    prec.pose.pose.orientation.y = 0.0;
    prec.pose.pose.orientation.z = 0.0;
    prec.pose.pose.orientation.w = 1.0;
    
    //on souscrit au topic /odom
    ros::Subscriber sub_odom  = n.subscribe("/odom", 1000, odomCallback);

    ros::Rate loop_rate(100);

    // Spin
    ros::spin();
    loop_rate.sleep();

    return 0;
}