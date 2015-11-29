#include "robotInfo.h"
#include <ros/ros.h>
#include <vector>
#include "comm_msg/Robot.h"
#include "comm_msg/RobotInfo.h"
#include "std_msgs/String.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotInfo_node");
	RobotInfo robotInfo;
	ros::Rate loop_rate(10);
    std::vector<comm_msg::Robot> tabRobots = robotInfo.getRobots();
	while (ros::ok())
    {
    	tabRobots = robotInfo.getRobots();
		ROS_INFO_COND(!tabRobots.empty(),"name: %s",tabRobots[0].name.c_str());
		ros::spinOnce();
		loop_rate.sleep();
    }
	return 0;
}
