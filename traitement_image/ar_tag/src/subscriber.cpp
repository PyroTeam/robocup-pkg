#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"


ros::Publisher pub_artag;

void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
	std_msgs::Int16 id;
   
   //ROS_INFO("test");
   if(msg->markers.size()!=0) {
   	ROS_INFO("I see: [%d]", msg->markers[0].id);
   	id.data = msg->markers[0].id;
   }
   	else{
   		id.data = -1;
   }
	pub_artag.publish(id);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "artag_reader");
	ros::NodeHandle n;
 	pub_artag = n.advertise<std_msgs::Int16>("/artag", 1000, 1);
 	ros::Rate r(10);
 	ROS_INFO("Subscribe");
	ros::Subscriber artag_topic = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1000, artagCallback);
	while(ros::ok()){
		r.sleep();
		ros::spinOnce();
	} 
    return 0;
  }