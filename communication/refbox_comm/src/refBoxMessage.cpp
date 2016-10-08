/**
 * \file 		refBoxMessage.cpp
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "refBoxMessage.h"
#include <iostream>

RefBoxMessage::RefBoxMessage(std::shared_ptr<protoMsg> msg, Periodicity p, double period, bool stopCond) : 
        m_message(msg), m_periodicity(p), m_period(period), m_stopCondition(stopCond), m_isFirstSend(true), m_callBackFunc(nullptr)
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
    m_callBackFunc = nullptr;
}

bool RefBoxMessage::isExpired()
{
//TODO
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

        //appel de la callback
        if (m_callBackFunc != nullptr)
        {
            bool ret = m_callBackFunc(*m_message);
            m_stopCondition = !ret;
        }
        m_isFirstSend = false;
        return true;
    }
    else
    {
        auto currentTime = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> nb_ms = (currentTime - m_lastSend);
        
        if (nb_ms.count() >= m_period)
        {
            //appel de la callback
            if (m_callBackFunc != nullptr)
            {
                bool ret = m_callBackFunc(*m_message);
                m_stopCondition = !ret;
            }
            m_lastSend = currentTime;
            return true;
        }
        return false;
    }
    
    
}
