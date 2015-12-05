#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

int main(int argc, char **argv){
	ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher artag_topic = n.advertise<ar_track_alvar::String>("computerVision/ar_pos_marker", 1000);
    ros::Rate loop_rate(10);

	while (ros::ok()){
    	std_msgs::String msg;
   		std::stringstream ss;
    	msg.data = ss.str();
	    ROS_INFO("%s", msg.data.c_str());
	    chatter_pub.publish(msg);	 	 
	    ros::spinOnce();	 
	    loop_rate.sleep();
	}
  return 0;
}