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
    laserScan laserData;

    ROS_INFO("Starting node line_detection_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "line_detection_node");
    
    ros::NodeHandle n;

    //on souscrit au topic /scan sur lequel les données laser sont transmises
    ros::Subscriber sub_laser  = n.subscribe("/scan", 1000, &laserScan::laserCallback, &laserData);

    //on publie les droites trouvées sur le topic /droites
    ros::Publisher Droites_pub = n.advertise<LineDetection::Modele>("/droites", 1000);

    //initialisation du random
    srand(time(NULL));

    //la copie dans la liste pourra peut-être se faire directement dans laserScan...
    //on copie le vector pCloud dans une liste
    std::list<Point>  listOfPoints = laserData.getPoints();
    std::list<Modele> listOfModeles = findLines(listOfPoints);
    
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
