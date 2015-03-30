#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "manager_msg/activity.h"

#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"
#include "gtServerSrv.h"
#include "LocaSubscriber.h"


using namespace std;

manager_msg::activity msgToGT(int n_robot)
{
       ROS_INFO("Starting GT Publisher");
       manager_msg::activity msg;  
       msg.nb_robot = n_robot;
       msg.state = manager_msg::activity::IN_PROGRESS; // Par exemple 
       return msg;

}

int main(int argc, char **argv) {

    ros::init(argc,argv,"executeur_taches");
    ros::NodeHandle n;
    int nb_robot;
    n.param<int>("robotNumber",nb_robot,0);  // nb robot, par défaut 0

/* Les machines */ 
    BaseStation bs;
    RingStation rs1; 
    RingStation rs2;
    CapStation cs1; 
    CapStation cs2;
    DeliveryStation ds; 
/* TEST */
    std::cout << "Hello !!" << std::endl;
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
 
/* Publisher topic générateur de taches */
    ros::Publisher activite_pub = n.advertise<manager_msg::activity>("/task_exec_state", 1000);
    manager_msg::activity msg;
    msg = msgToGT(nb_robot);
    activite_pub.publish(msg);

/* Subscriber topic localisation */
    LocaSubscriber loca_sub;
    ros::Subscriber sub = n.subscribe("Subscriber",1000,&LocaSubscriber::tesCallback, &loca_sub);

/* Let's Spin until the end of the world !! */
    ros::spin();
    return 0;
  
}