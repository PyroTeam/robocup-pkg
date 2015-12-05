#include "ExploInfoSubscriber.h"

ExploInfoSubscriber::ExploInfoSubscriber()
{
	ros::NodeHandle n;
	m_sub = n.subscribe("refBoxComm/ExplorationInfo",1000,&ExploInfoSubscriber::tesCallback, this);
}

ExploInfoSubscriber::~ExploInfoSubscriber(){}

void ExploInfoSubscriber::tesCallback(const comm_msg::ExplorationInfo &msg)
{
	ROS_INFO("I heard the ExplorationInfo publisher ");

	m_signals = msg.signals;
	m_zones   = msg.zones;
}

void ExploInfoSubscriber::interpretationFeu()
{
	int i=0, j=0;
	comm_msg::LightSpec light;
    uint8_t r_state,g_state,y_state;
    for(i = 0;i<lSpec.size() ;i++)
    {
    	if(lSpec[i].color == light.RED)    r_state = lSpec[i].state;
    	if(lSpec[i].color == light.YELLOW) y_state = lSpec[i].state;
    	if(lSpec[i].color == light.GREEN)  g_state = lSpec[i].state;
    }
    
    j=0;
	while(j<m_signals.size())
	{
		i=0;
		while(i<m_signals[j].lights.size())
		{
			if(m_signals[j].lights[i].color == light.RED)
			{
				if(r_state == m_signals[j].lights[i].state) i++;
				else break;					
			}

			else if(m_signals[j].lights[i].color == light.GREEN)
			{		
				if(g_state == m_signals[j].lights[i].state) i++;
				else break;
			}
			else if(m_signals[j].lights[i].color == light.YELLOW)
			{
				if(y_state == m_signals[j].lights[i].state) i++;
				else break;
			}
		}
		if(i=3) break;
		j++;
    } 
    this->type = m_signals[j].type;
    for(i = 0 ; i < lSpec.size() ; i++)
    {
	 	ROS_INFO("color : %d , state : %d",lSpec[i].color,lSpec[i].state);
	}
	ROS_INFO("type : %s", this->type.c_str());
}

