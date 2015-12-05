/* Fake Localisation Publisher Node */

#include "ros/ros.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/LightSpec.h"
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker_ei");

	ROS_INFO("Starting publishing ");

	ros::NodeHandle n;

	ros::Publisher ei_pub = n.advertise<comm_msg::ExplorationInfo>("refBoxComm/ExplorationInfo", 1000);

	ros::Rate loop_rate(1);

	while (ros::ok())
	{
		comm_msg::ExplorationInfo msg;
		comm_msg::LightSpec light;

		std::vector<comm_msg::ExplorationSignal>  m_signals(2);

		m_signals[0].type="dgDH567q";

		light.color = light.RED;
		light.state = light.BLINK;
		m_signals[0].lights.push_back(light);

		light.color = light.GREEN;
		light.state = light.OFF;
		m_signals[0].lights.push_back(light);

		light.color = light.YELLOW;
		light.state = light.ON;
		m_signals[0].lights.push_back(light);

		m_signals[1].type="w-'V(6gh";

		light.color = light.RED;
		light.state = light.OFF;
		m_signals[1].lights.push_back(light);

		light.color = light.GREEN;
		light.state = light.ON;
		m_signals[1].lights.push_back(light);

		light.color = light.YELLOW;
		light.state = light.ON;
		m_signals[1].lights.push_back(light);

		std::vector<comm_msg::ExplorationZone>  m_zones(2);
		m_zones[0].zone = 0;
		m_zones[0].team_color = 0;
		m_zones[1].zone = 1;
		m_zones[1].team_color = 1;
		msg.signals = m_signals;
		msg.zones = m_zones;
		ei_pub.publish(msg);
		ROS_INFO("publishing type 0 : %s", msg.signals[0].type.c_str());
		ROS_INFO("publishing type 1 : %s", msg.signals[1].type.c_str());
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
