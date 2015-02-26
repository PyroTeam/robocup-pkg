#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <limits>
#include <vector>

#include "laserscan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"

int main(int argc, char** argv)
{
    ROS_INFO("Starting node line_detection_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "line_detection_node");
    
    ros::NodeHandle n;

    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, laserCallback);

    ros::Publisher Droites_pub = n.advertise<LineDetection::Modele>("/droites", 1000);

    //qqch

    laserScan laserData;

    ros::Rate loop_rate (100);

    while(n.ok())
    {
        // Publish
        Droites_pub.publish(Droites_msg);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
