#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "trait_im_msg/artag.h"

std_msgs::Int16 id;

bool giveId(trait_im_msg::artag::Request &req,trait_im_msg::artag::Response &res){
  res.id = id.data;
  return true;
}


void artagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
   ROS_INFO("test");
   if(msg->markers.size()!=0){
   	ROS_INFO("I see: [%d]", msg->markers[0].id);
   	id.data = msg->markers[0].id;
   }
   	else{
   		id.data = -1;
   }
}

int main(int argc, char **argv){
	ros::init(argc, argv, "artag_reader");
	ros::NodeHandle n;
 	ros::ServiceServer service = n.advertiseService("/artag", giveId);
 	ros::Rate r(10);
 	ROS_INFO("Subscribe");
	ros::Subscriber artag_topic = n.subscribe<ar_track_alvar::AlvarMarkers>("/ar_pose_marker", 1000, artagCallback);
	while(ros::ok()){
		r.sleep();
		ros::spinOnce();
	} 
  return 0;
}