#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "deplacement_msg/Landmarks.h"
#include "landmarks_detection_utils.h"
#include "conversion_functions.h"
#include "Segment.h"
#include "LaserScan.h"
#include "common_utils/Zone.h"

using namespace common_utils;

int main(int argc, char** argv)
{
    // Objet laser
    LaserScan laserData;

    //Initialisation du noeud ROS
    ros::init(argc, argv, "landmarks_extraction_node");
    ROS_INFO("Starting node landmarks_extraction_node");

    ros::NodeHandle n;
    std::string tf_prefix;
    n.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    // Subscribe to LaserScan
    ros::Subscriber sub_laser  = n.subscribe("hardware/scan_pcl", 1, &LaserScan::pclCallback, &laserData);

    // Publish found segments and machines
    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("objectDetection/machines", 1000);
    ros::Publisher pub_segments = n.advertise< deplacement_msg::Landmarks >("objectDetection/segments", 1000);

    // Initialisation du random
    srand(time(NULL));

    // Le noeud tourne à 10Hz
    ros::Rate loop_rate (10);
    while(n.ok())
    {
        // Trouve les lignes
        std::list<Model>  listOfModels      = findLines(laserData.getPoints(), 10, 0.05, 10);
        // Construit les segments
        std::list<Segment> listOfSegments   = buildSegmentsFromModels(listOfModels);
        // Reconnaît les machines à partir des segments
        std::vector<Machine> listOfMachines = recognizeMachinesFrom(listOfSegments);

        deplacement_msg::Landmarks machines;
        deplacement_msg::Landmarks segments;

        for (auto &it : listOfMachines)
        {
            machines.landmarks.push_back(it.pose());
        }
        machines.header = laserData.getHeader();

        for (auto &it : listOfSegments)
        {
            segments.landmarks.push_back(pointToPose2D(it.getMin()));
            segments.landmarks.push_back(pointToPose2D(it.getMax()));
        }
        segments.header = laserData.getHeader();

        // Publish
        pub_segments.publish(segments);
        pub_machines.publish(machines);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
