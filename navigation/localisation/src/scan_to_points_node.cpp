#include "ros/ros.h"
#include "deplacement_msg/Points.h"
#include "deplacement_msg/Droites.h"
#include "sensor_msgs/LaserScan.h"
#include "line_detection_utils.h"
#include <vector>
#include <cmath>
#include "laserScan.h"

int main(int argc, char** argv)
{
    ROS_INFO("Start noeud scanToPoints");

    laserScan laserData;

    //Initialisation du noeud ROS
    ros::init(argc, argv, "scanToPoints");
    
    ros::NodeHandle n;

    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, &laserScan::laserCallback, &laserData);

    ros::Publisher points_pub = n.advertise<deplacement_msg::Points>("/points", 1000);

    ros::Rate loop_rate (100);

    while(n.ok())
    {
        deplacement_msg::Points msgPoints;
        msgPoints = convertPtsToDeplMsgPts(laserData.getPoints());

        // Publish
        points_pub.publish(msgPoints);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
