/**
 * \file 		refBoxMessage.cpp
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "refBoxMessage.h"

RefBoxMessage::RefBoxMessage(std::shared_ptr<protoMsg> msg, Periodicity p, double period, bool stopCond) : 
        m_message(msg), m_periodicity(p), m_period(p), m_stopCondition(stopCond), m_isFirstSend(true)
{

}

RefBoxMessage::~RefBoxMessage()
{

}

void RefBoxMessage::set(std::shared_ptr<protoMsg> msg, Periodicity p, double period, bool stopCond)
{
    m_message = msg;
    m_periodicity = p;
    m_period = period;
    m_stopCondition = stopCond;
    m_isFirstSend = true;
}

bool RefBoxMessage::isExpired()
{
/*  
  if(m_periodicity == NON_PERIODIC)
    {
        
    }
    else if (m_periodicity == PERIODIC)
    {
        return m_stopCondition;
    }

    return true;
    */
    
    return m_stopCondition;
}

bool RefBoxMessage::isReady()
{
    if(m_isFirstSend)
    {
        switch(m_periodicity)
        {
        case NON_PERIODIC:
            m_stopCondition = true;
        break;
        case PERIODIC:
        default:
            m_lastSend = std::chrono::system_clock::now();
        break;
        }

        //TODO appel de la callback
    
        m_isFirstSend = false;
        return true;
    }
    else
    {
        auto currentTime = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> nb_ms = (currentTime - m_lastSend);
        if (nb_ms.count() >= m_period)
        {
            
            //TODO appel de la callback    
            
            m_lastSend = currentTime;
            return true;
        }
        return false;
    }
    
    
}
