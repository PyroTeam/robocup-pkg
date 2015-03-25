#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "laserScan.h"

#include "landmarks_detection_utils.h"

int main(int argc, char** argv)
{
    laserScan laserData;

    ROS_INFO("Starting node landmarks_extraction_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "landmarks_extraction_node");
    
    ros::NodeHandle n;

    //on souscrit au topic /fake_scan sur lequel les données laser sont transmises
    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, &laserScan::laserCallback, &laserData);

    //on publie les machines trouvées sur le topic /machines
    //et les segments sur le topic /segments
    ros::Publisher pub_machines = n.advertise< deplacement_msg::Landmarks >("/machines", 1000);
    ros::Publisher pub_segments = n.advertise< deplacement_msg::Landmarks >("/segments", 1000);

    //initialisation du random
    srand(time(NULL));

    ros::Rate loop_rate (5);

    while(n.ok())
    {
        const std::list<Point> &listOfPoints    = laserData.getPoints();
        std::list<Modele>  listOfModeles        = findLines(listOfPoints, 30, 0.1, 20);
        //voir amélioration avec la fonction convertModelesIntoMachines
        std::list<Segment> listOfSegments       = buildSegments(listOfModeles);
        std::vector<Machine> listOfMachines     = recognizeMachinesFrom(listOfSegments);
        
        deplacement_msg::Landmarks machines;
        deplacement_msg::Landmarks segments;

        for (auto &it : listOfMachines){ 
            machines.landmarks.push_back(it.getCentre());
        }
        for (auto &it : listOfSegments){ 
            segments.landmarks.push_back(pointToPose2D(it.getMin()));
            segments.landmarks.push_back(pointToPose2D(it.getMax()));
        }

        // Publish
        pub_machines.publish(machines);
        pub_segments.publish(segments);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}