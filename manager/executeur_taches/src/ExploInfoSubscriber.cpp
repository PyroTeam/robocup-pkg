#include "ExploInfoSubscriber.h"

ExploInfoSubscriber::ExploInfoSubscriber()
{
    ros::NodeHandle n;
    m_sub = n.subscribe("refBoxComm/ExplorationInfo",1000,&ExploInfoSubscriber::tesCallback, this);
}

ExploInfoSubscriber::~ExploInfoSubscriber(){}

void ExploInfoSubscriber::tesCallback(const comm_msg::ExplorationInfo &msg)
{
    ROS_INFO_ONCE("I heard the ExplorationInfo publisher ");

    m_signals = msg.signals;
    m_zones   = msg.zones;
}

void ExploInfoSubscriber::interpretationFeu()
{
    int i=0, j=0;
    comm_msg::LightSpec light;
    uint8_t r_state,g_state,y_state;

    for(i = 0; i < m_lSpec.size(); i++)
    {
        if(m_lSpec[i].color == light.RED)
        {
            r_state = m_lSpec[i].state;
        }
        if(m_lSpec[i].color == light.YELLOW)
        {
            y_state = m_lSpec[i].state;
        }
        if(m_lSpec[i].color == light.GREEN)
        {
            g_state = m_lSpec[i].state;
        }
    }

    bool found = false;

    while(j < m_signals.size())
    {
        i=0;
        while(i < m_signals[j].lights.size())
        {
            if(m_signals[j].lights[i].color == light.RED)
            {
                if(r_state == m_signals[j].lights[i].state)
                {
                    i++;
                }
                else
                {
                    break;
                }
            }
            else if(m_signals[j].lights[i].color == light.GREEN)
            {
                if(g_state == m_signals[j].lights[i].state)
                {
                    i++;
                }
                else
                {
                    break;
                }
            }
            else if(m_signals[j].lights[i].color == light.YELLOW)
            {
                if(y_state == m_signals[j].lights[i].state)
                {
                    i++;
                }
                else
                {
                    break;
                }
            }
        }

        if(i == 3)
        {
            found = true;
            break;
        }
        
        j++;
    }
    if (!found)
    {
        this->type = "";
        ROS_ERROR("Signal not found");
        return;
    }

    ROS_INFO_NAMED("interpretationFeu", "i:%d j:%d size:%d", i, j, int( m_signals.size()));
    ROS_INFO_NAMED("interpretationFeu", "type: %s", m_signals[j].type.c_str());
    this->type = m_signals[j].type;

    for(i = 0 ; i < m_lSpec.size() ; i++)
    {
        ROS_INFO("color : %d , state : %d",m_lSpec[i].color,m_lSpec[i].state);
    }
    ROS_INFO("type : %s", this->type.c_str());

}
