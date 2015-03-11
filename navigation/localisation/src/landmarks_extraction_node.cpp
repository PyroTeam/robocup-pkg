#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <limits>
#include <vector>

#include "laserscan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Machine.h"
#include "line_detection_utils.h"
#include "machine_detection_utils.h"


int main(int argc, char** argv)
{
    laserScan laserData;

    ROS_INFO("Starting node landmarks_extraction_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "landmarks_extraction_node");
    
    ros::NodeHandle n;

    //on souscrit au topic /scan sur lequel les données laser sont transmises
    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, &laserScan::laserCallback, &laserData);

    //on publie les droites trouvées sur le topic /droites
    ros::Publisher pub_machines = n.advertise<geometry_msg::Pose2D>("/machines", 1000);

    //initialisation du random
    srand(time(NULL));

    const std::list<Point> &listOfPoints    = laserData.getPoints();
    std::list<Modele>  listOfModeles        = findLines(listOfPoints);
    std::list<Segment> listOfSegments       = buildSegments(listOfModeles);
    std::list<Machine> listOfMachines       = recognizeMachinesFrom(listOfSegments);

    
    ros::Rate loop_rate (100);

    while(n.ok())
    {
        for (auto &it : listOfMachines){
            geometry_msg::Pose2D msgMachine;
            msgMachine = convertMachineToPose2D(it);

            // Publish
            pub_machines.publish(msgMachines);
        }
        

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}