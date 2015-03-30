#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"
#include "gtServerSrv.h"
#include "LocaSubscriber.h"


using namespace std;

int main(int argc, char **argv) {

    ros::init(argc,argv,"executeur_taches");
    ros::NodeHandle n;

/* Les machines */ 
    BaseStation bs;
    RingStation rs1; 
    RingStation rs2;
    CapStation cs1; 
    CapStation cs2;
    DeliveryStation ds; 
/* TEST */
    std::cout << "Hello Sandra!!" << std::endl;
    std::cout << "Test avant : " << bs.getEntreeMachine().x <<" "<< bs.getEntreeMachine().y << std::endl;
    
    geometry_msgs::Pose2D monPoint;
    monPoint.x = 3.2;
    monPoint.y = 4.5;
    monPoint.theta = 0.4;
    bs.majEntree(monPoint);
    std::cout << "Test apres : " << bs.getEntreeMachine().x <<" "<< bs.getEntreeMachine().y << std::endl;
/* FIN TEST */ 

/* service reponse au générateur de taches */
    gtServerSrv gtsrv;
    gtsrv.setId(1);
    ros::ServiceServer service = n.advertiseService("order", &gtServerSrv::responseToGT, &gtsrv);
    ROS_INFO("I'm READY ! ");
    
/* topic localisation */
    LocaSubscriber loca_sub;
    ros::Subscriber sub = n.subscribe("Subscriber",1000,&LocaSubscriber::tesCallback, &loca_sub);

    ros::spin();
    return 0;
  
}