#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "deplacement_msg/Landmarks.h"
#include "landmarks_detection_utils.h"
#include "conversion_functions.h"
#include "Segment.h"
#include "LaserScan.h"

int main(int argc, char** argv)
{
    // Objet laser (convertit et contient les points laser en coord cartesiennes)
    laserScan laserData;

    //Initialisation du noeud ROS
    ros::init(argc, argv, "LandmarksExtraction_node");
    ROS_INFO("Starting node LandmarksExtraction_node");

    ros::NodeHandle n;
    std::string tf_prefix;
    n.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    // Subscribe to laserScan
    ros::Subscriber sub_laser  = n.subscribe("hardware/scan", 1, &laserScan::laserCallback, &laserData);

    // Publish found segments and machines
    //ros::Publisher pub_droites  = n.advertise< deplacement_msg::Landmarks >("objectDetection/droites", 1000);
    //ros::Publisher pub_segments = n.advertise< deplacement_msg::Landmarks >("objectDetection/segments", 1000);
    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("objectDetection/machines", 1000);

    // Initialisation du random
    srand(time(NULL));

    // Le noeud tourne à 10Hz
    ros::Rate loop_rate (10);
    while(n.ok())
    {
        // Enregistre une copie du scan en coord cartesiennes
        const std::list<geometry_msgs::Point> &listOfPoints    = laserData.getPoints();
        // Trouve les lignes, stocke les points restants dans l
        std::list<geometry_msgs::Point> l;
        std::list<Model>  listOfModels      = findLines(listOfPoints, 20, 0.05, 20, l);
        // Construit les segments
        std::list<Segment> listOfSegments   = buildSegmentsFromModels(listOfModels);

        std::vector<Machine> listOfMachines = recognizeMachinesFrom(listOfSegments);

        //deplacement_msg::Landmarks droites;
        //deplacement_msg::Landmarks segments;
        deplacement_msg::Landmarks machines;
/*
        for (auto &it : listOfModels)
        {
            droites.landmarks.push_back(it.getLine().getPoint());
            droites.header.stamp = laserData.getTime();
            droites.header.frame_id = tf_prefix+"laser_link";
        }
        for (auto &it : listOfSegments)
        {
            segments.landmarks.push_back(pointToPose2D(it.getMin()));
            segments.landmarks.push_back(pointToPose2D(it.getMax()));
            segments.header.stamp = laserData.getTime();
            segments.header.frame_id = tf_prefix+"laser_link";
        }*/
        for (auto &it : listOfMachines)
        {
            machines.landmarks.push_back(it.getCentre());
            machines.header.stamp = laserData.getTime();
            machines.header.frame_id = tf_prefix+"laser_link";
        }

        // Publish
        //pub_segments.publish(segments);
        //pub_droites.publish(droites);
        pub_machines.publish(machines);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
