#include "ros/ros.h"
#include <manager_msg/artag.h>


bool test_arTag(manager_msg::artag::Request &req,manager_msg::artag::Response &res){
	res.id = 34;
	return true;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"artag_server");
	ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("artag", test_arTag);
    
	ros::spin();
	return 0;
}