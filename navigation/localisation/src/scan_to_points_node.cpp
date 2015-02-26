#include "ros/ros.h"
#include "laserScan.h"
#include "deplacement_msg/Points.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cmath>

std::vector<deplacement_msg::Point> convertPtToDeplMsgPt(const std::vector<Point> &points){
    std::vector<deplacement_msg::Point> punkt;
    punkt.resize(points.size());
    for(auto p:points)
    {
        deplacement_msg::Point pt;
        pt.x=p.x;
        pt.y=p.y;
        punkt.push_back(pt);
    }

    return punkt;
}

int main(int argc, char** argv)
{
    ROS_INFO("Start noeud scanToPoints");

    laserScan laserData;

    //Initialisation du noeud ROS
    ros::init(argc, argv, "scanToPoints");
    
    ros::NodeHandle n;

    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, &laserScan::laserCallback, &laserData);

    ros::Publisher Points_pub = n.advertise<deplacement_msg::Points>("/points", 1000);

    ros::Rate loop_rate (100);

    while(n.ok())
    {
        deplacement_msg::Points msgPoints;
        msgPoints.points=convertPtToDeplMsgPt(laserData.getPoints());

        // Publish
        Points_pub.publish(msgPoints);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
