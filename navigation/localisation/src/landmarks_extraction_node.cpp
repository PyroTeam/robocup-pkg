#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "deplacement_msg/Landmarks.h"
#include "laserScan.h"
#include "landmarks_detection_utils.h"

int main(int argc, char** argv)
{
    // Objet laser (convertit et contient les points laser en coord carthesiennes)
    laserScan laserData;

    //Initialisation du noeud ROS
    ros::init(argc, argv, "landmarks_extraction_node");
    ROS_INFO("Starting node landmarks_extraction_node");
    
    ros::NodeHandle n;

    // Subscribe to laserScan
    ros::Subscriber sub_laser  = n.subscribe("/scan", 1, &laserScan::laserCallback, &laserData);

    // Publish found segments and machines
    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/machines", 1000);
    ros::Publisher pub_segments = n.advertise< deplacement_msg::Landmarks >("/segments", 1000);
    ros::Publisher pub_droites = n.advertise< deplacement_msg::Landmarks >("/droites", 1000);

    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("/visualization_markers", 10000);

    // Initialisation du random
    srand(time(NULL));

    // Le noeud tourne  25Hz
    ros::Rate loop_rate (25);
    while(n.ok())
    {
        // Transforme les points scans de coord polaires en coord carthesiennes
        const std::list<Point> &listOfPoints    = laserData.getPoints();
        // Trouve les lignes, stock les points restants
        std::list<Point> l;
        std::list<Modele>  listOfModeles        = findLines(listOfPoints, 30, 0.1, 30, l);
        // Construit les segments
        std::list<Segment> listOfSegments       = buildSegments(listOfModeles);
        // Trouve les machines
        std::vector<Machine> listOfMachines     = recognizeMachinesFrom(listOfSegments);


        // Construction des messages
        deplacement_msg::Landmarks machines;
        machines.header.frame_id="laser_link";
        machines.header.stamp = laserData.m_stamp;
        for (auto &it : listOfMachines){ 
            machines.landmarks.push_back(it.getCentre());
        }

        deplacement_msg::Landmarks segments;
        segments.header.frame_id="laser_link";
        segments.header.stamp = laserData.m_stamp;
        for (auto &it : listOfSegments){ 
            segments.landmarks.push_back(pointToPose2D(it.getMin()));
            segments.landmarks.push_back(pointToPose2D(it.getMax()));
        }        

        deplacement_msg::Landmarks droites;
        droites.header.frame_id="laser_link";
        droites.header.stamp = laserData.m_stamp;
        for (auto &it : listOfModeles){ 
            // Construit les points de la droite
            Point pointA = it.getPoints().front();
            Point pointB = it.getPoints().back();

            droites.landmarks.push_back(pointToPose2D(pointA));
            droites.landmarks.push_back(pointToPose2D(pointB));
        }

        int cpt = 0;
        for (auto &it : listOfModeles)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id="laser_link";
            marker.header.stamp = laserData.m_stamp;
            // Segments
            marker.header.frame_id = "/laser_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "visualisation_modeles";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = 10+cpt++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;

            marker.scale.x = 0.1;

            marker.color.g = 1.0;
            marker.color.a = 0.3;

            for (auto &pt : it.getPoints())
            {
                geometry_msgs::Point p;
                p.x = pt.getX();
                p.y = pt.getY();
                p.z = 0;

                marker.points.push_back(p);
            }

            // markers_pub.publish(marker);
        }

        // Publish
        pub_machines.publish(machines);
        pub_segments.publish(segments);
        pub_droites.publish(droites);


        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}