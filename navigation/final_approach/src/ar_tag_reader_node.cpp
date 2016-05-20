#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

ros::Publisher pubMvt;
float positionX = 0;
float positionZ = 0;
float orientationZ = 0;
void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
	geometry_msgs::Twist msgTwist;
	if(msg->markers.size()!=0) 
	{
   		ROS_INFO("I see: [%d]", msg->markers[0].id);
		float px = msg->markers[0].pose.pose.position.x;
		float py = msg->markers[0].pose.pose.position.y;
		float pz = msg->markers[0].pose.pose.position.z;
		float ox = msg->markers[0].pose.pose.orientation.x;
		float oy = msg->markers[0].pose.pose.orientation.y;
		float oz = msg->markers[0].pose.pose.orientation.z;
		ROS_INFO("position: %f %f %f",px,py,pz);
		ROS_INFO("orientation: %f %f %f",ox,oy,oz);
		if(std::abs(px) < 0.005)
                {
                        msgTwist.linear.y = 0;
                }
                else
                {
                        msgTwist.linear.y = -0.75*px;
                }

		if(std::abs(pz-0.50) < 0.01)
		{
			msgTwist.linear.x = 0;
		}
		else
		{
			msgTwist.linear.x = 0.25*(pz-0.50);
		}
		if(std::abs(oz) < 0.01)
                {
                        msgTwist.angular.z = 0;
                }
                else
                {
                        msgTwist.angular.z = 0.125*oz;
                }
	}
	else
	{
		msgTwist.linear.x=0;
		msgTwist.linear.y=0;
		msgTwist.angular.z=0;
	}
	pubMvt.publish(msgTwist);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "artag_reader_node");
	ros::NodeHandle n;
 	ros::Rate r(100);
 	ROS_INFO("Subscribe");
	ros::Subscriber artag_topic = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("computerVision/ar_pose_marker", 1000, artagCallback);
	pubMvt = n.advertise<geometry_msgs::Twist>("hardware/cmd_vel",1000);
	while(ros::ok())
	{
			
		r.sleep();
		ros::spinOnce();
	} 
    return 0;
  }
