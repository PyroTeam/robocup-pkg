#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "manager_msg/activity.h"

#include "GtServerSrv.h"
#include "LocaSubscriber.h"
#include "Machine.h"


using namespace std;

int main(int argc, char **argv) {

    ros::init(argc,argv,"executeur_taches");
    ros::NodeHandle n;
    int nb_robot;
    n.param<int>("robotNumber",nb_robot,0);  // nb robot, par défaut 0

/* service reponse au générateur de taches */
    GtServerSrv gtsrv;
    ROS_INFO("I'm READY ! ");
    gtsrv.setId(1);
    ros::ServiceServer service = n.advertiseService("order", &GtServerSrv::responseToGT, &gtsrv);
    
 
/* Publisher topic générateur de taches */
    ros::Publisher activite_pub = n.advertise<manager_msg::activity>("/task_exec_state", 1000);
    activite_pub.publish(gtsrv.getActivityMsg()); 

/* Subscriber topic localisation */
    LocaSubscriber loca_sub;
    ros::Subscriber sub = n.subscribe("/landmarks",1000,&LocaSubscriber::tesCallback, &loca_sub);

/* Let's Spin until the end of the world !! */
    ros::spin();
    return 0;
  
}