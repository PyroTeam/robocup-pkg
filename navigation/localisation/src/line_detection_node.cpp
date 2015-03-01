#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <limits>
#include <vector>

#include "laserscan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "line_detection_utils.h"


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
    ros::Publisher Droites_pub = n.advertise<deplacement_msg::Droites>("/droites", 1000);

    //initialisation du random
    srand(time(NULL));

    std::list<Point>  listOfPoints = laserData.getPoints();
    std::list<Modele> listOfModeles = findLines(listOfPoints);
    
    ros::Rate loop_rate (100);

    while(n.ok())
    {
        deplacement_msg::Droites msgDroites;
        msgDroites = convertModelesToDeplMsgDroite(const std::list<Modele> &modeles);

        // Publish
        Droites_pub.publish(msgDroites);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
