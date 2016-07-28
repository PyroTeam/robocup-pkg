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

    ros::Subscriber sub_machines  = n.subscribe("objectDetection/machines", 1, &EKF::machinesCallback, &ekf);

    ros::Publisher pub_robot    = n.advertise<deplacement_msg::Robot>("objectDetection/robot", 1);
    ros::Publisher pub_machines = n.advertise<deplacement_msg::Machines>("objectDetection/landmarks", 1);

    ros::Rate loop_rate(30);

    int cpt = 0;

    while (n.ok())
    {
        if(ekf.initOdom())
        {
            ekf.prediction();

            //pour toutes les machines observées
            for (auto &it : ekf.getTabMachines())
            {
                //si on a déjà vu cette machine
                int index = ekf.checkStateVector(it);
                if (index != 0)
                {
                    //on corrige sa position et on se recale par rapport à celle ci
                    ekf.correction(it, index);
                }
                else
                {
                    // on ajoute cette machine
                    std::cout << "ajout machine" << std::endl;
                    ekf.addMachine(it);
                    cpt++;
                }
            }

            std::cout << "machine(s) ajoutée(s) = " << cpt << "\n" << std::endl;

            VectorXd xMean = ekf.getXmean();
            //std::cout << "xMean : \n" << xMean << std::endl;

            ekf.printAreas();

            deplacement_msg::Robot robot;
            robot.pose.x = xMean(0);
            robot.pose.y = xMean(1);
            robot.pose.theta = xMean(2);
            robot.header.stamp = ros::Time::now();
            robot.header.frame_id = tf_prefix+"odom";

            deplacement_msg::Machines m;
            for (int i = 3; i < xMean.rows(); i = i + 3)
            {
                geometry_msgs::Pose2D md;
                md.x     = xMean(i);
                md.y     = xMean(i+1);
                md.theta = xMean(i+2);
                deplacement_msg::MPS mps;
                mps.pose = md;
                m.landmarks.push_back(mps);
            }

            pub_robot.publish(robot);
            pub_machines.publish(m);

            m.landmarks.clear();
        }

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
