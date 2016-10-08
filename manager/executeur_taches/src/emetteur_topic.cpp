#include "ros/ros.h"
#include "std_msgs/String.h"
#include "manager_msg/activity.h" 
#include "manager_msg/GameState.h"

#include <sstream>

using namespace manager_msg;

int main(int argc, char **argv)
{
	ROS_INFO("Starting node emetteur_topic game_state");

	ros::init(argc, argv, "emetteur_topic");

	ros::NodeHandle n;

	ros::Publisher activite = n.advertise<activity>("manager/task_exec_state", 1000);
	ros::Publisher etatdujeu = n.advertise<GameState>("manager/game_state", 1000);

	ros::Rate loop_rate(1);

	int count = 0;
	while (ros::ok())
	{
		activity msg;
		GameState msg2;
		msg2.state=GameState::RUNNING;

		msg.state = activity::END; /* Par exemple */
		if(count>10)
		{
			msg.machine_used = activity::DS;
			msg2.phase=GameState::PRODUCTION;
		}
		else 
		{
			msg.machine_used = activity::CS1;
			msg2.phase=GameState::EXPLORATION;
		}
		
		activite.publish(msg);
		etatdujeu.publish(msg2);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
