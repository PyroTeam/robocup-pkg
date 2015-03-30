/* A SUPPRIMER !!! */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>


#include "gtServerSrv.h"


using namespace std;

int main(int argc, char **argv) {
/* TESTS */ 

	ros::init(argc,argv,"responding_Generateur_Taches");
	ros::NodeHandle n;
    gtServerSrv gtsrv;
    gtsrv.setId(1);
	ros::ServiceServer service = n.advertiseService("order", &gtServerSrv::responseToGT, &gtsrv);
	ROS_INFO("I'm READDY ! ");
	ros::spin();
	
	return 0;
  
}