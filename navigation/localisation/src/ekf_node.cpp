#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <cmath>
#include "deplacement_msg/Landmarks.h"
#include "deplacement_msg/Machines.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "LaserScan.h"
#include "landmarks_detection_utils.h"

#include "Ekf.h"

using namespace Eigen;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Ekf_node");

    ros::NodeHandle n;
    std::string tf_prefix;
    n.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    EKF ekf;
    tf::TransformListener tf_listener;
    ekf.setTF(&tf_listener);

    ros::Publisher pub_robot    = n.advertise<deplacement_msg::Robot>("objectDetection/robot", 1);
    ros::Publisher pub_machines = n.advertise<deplacement_msg::Machines>("objectDetection/landmarks", 1);

    ros::Rate loop_rate(30);

    int cpt = 0;

    while (n.ok())
    {
        if(ekf.initOdom())
        {
            ekf.run();

            deplacement_msg::Robot robot;
            robot.pose = ekf.getRobot();
            robot.header.stamp = ros::Time::now();
            robot.header.frame_id = tf_prefix+"odom";

            deplacement_msg::Machines m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = tf_prefix+"odom";
            for (auto &it : ekf.getLandmarks())
            {
                deplacement_msg::MPS mps;
                mps.pose = it;
                m.landmarks.push_back(mps);
            }

            pub_robot.publish(robot);
            pub_machines.publish(m);
        }

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
