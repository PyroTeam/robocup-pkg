/* Fake Localisation Publisher Node */

#include "ros/ros.h"
#include "manager_msg/ExplorationInfo.h"
#include "manager_msg/LightSpec.h"
#include <string>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ROS_INFO("Starting publishing ");

  ros::NodeHandle n;

  ros::Publisher ei_pub = n.advertise<manager_msg::ExplorationInfo>("/explorationInfo", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    
    ROS_INFO("testing");
    manager_msg::ExplorationInfo msg;
    manager_msg::LightSpec light;

    std::vector<manager_msg::ExplorationSignal>  m_signals(2);

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

    m_signals[1].type="dgDDV67q";

    light.color = light.RED;
    light.state = light.OFF;
    m_signals[1].lights.push_back(light);

    light.color = light.GREEN;
    light.state = light.ON;
    m_signals[1].lights.push_back(light);

    light.color = light.YELLOW;
    light.state = light.ON;
    m_signals[1].lights.push_back(light);

    std::vector<manager_msg::ExplorationZone>  m_zones(2);
    m_zones[0].zone = 0;
    m_zones[0].team_color = 0;
    m_zones[1].zone = 1;
    m_zones[1].team_color = 1;
    msg.signals = m_signals;
    msg.zones = m_zones;
    ei_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}