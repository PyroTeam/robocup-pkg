#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Machines.h"
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
    ros::Subscriber sub_laser  = n.subscribe(   "/fake_scan",
                                                1000,
                                                &laserScan::laserCallback,
                                                &laserData);

    //on publie les machines trouvées sur le topic /machines
    ros::Publisher pub_machines = n.advertise< deplacement_msg::Machines >("/machines", 1000);

    //initialisation du random
    srand(time(NULL));

    ros::Rate loop_rate (2);

    while(n.ok())
    {
        const std::list<Point> &listOfPoints    = laserData.getPoints();
        std::list<Modele>  listOfModeles        = findLines(listOfPoints);
        std::list<Segment> listOfSegments       = buildSegments(listOfModeles);
        std::vector<Machine> listOfMachines     = recognizeMachinesFrom(listOfSegments);
        
        deplacement_msg::Machines machines;

        for (auto &it : listOfMachines){ 
            machines.machines.push_back(it.getCentre());
        }

        // Publish
        pub_machines.publish(machines);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}