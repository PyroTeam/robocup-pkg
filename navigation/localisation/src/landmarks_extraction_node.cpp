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
    ros::init(argc, argv, "landmarks_extraction_node");
    ROS_INFO("Starting node landmarks_extraction_node");
    
    ros::NodeHandle n;

    // Subscribe to laserScan
    ros::Subscriber sub_laser  = n.subscribe("hardware/scan", 1, &laserScan::laserCallback, &laserData);

    // Publish found segments and machines
    ros::Publisher pub_droites  = n.advertise< deplacement_msg::Landmarks >("objectDetection/droites", 1000);
    ros::Publisher pub_segments = n.advertise< deplacement_msg::Landmarks >("objectDetection/segments", 1000);

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

        deplacement_msg::Landmarks droites;
        deplacement_msg::Landmarks segments;

        for (auto &it : listOfModels)
        { 
            droites.landmarks.push_back(it.getLine().getPoint());
        }
        for (auto &it : listOfSegments)
        { 
            segments.landmarks.push_back(pointToPose2D(it.getMin()));
            segments.landmarks.push_back(pointToPose2D(it.getMax()));
        }
        // Publish
        pub_segments.publish(segments);
        pub_droites.publish(droites);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}