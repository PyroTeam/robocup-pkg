/**
 * \file 		sendScheduler.cpp
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include <iostream>
#include <chrono>

#include "refBoxTransport.h"
#include "sendScheduler.h"

SendScheduler::SendScheduler(): 
    m_timeSlot(10), m_isOpen(false), m_transport(nullptr)
{

}

SendScheduler::SendScheduler(std::shared_ptr<RefBoxTransport> &refBoxTr): 
    m_timeSlot(10), m_isOpen(false), m_transport(refBoxTr)
{

}

SendScheduler::~SendScheduler()
{
	if(m_isOpen)
	{
		close();
	}
}

void SendScheduler::close()
{
	m_isOpen = false;
	m_t->join();
}



void SendScheduler::run()
{
    m_isOpen = true;
	while (m_isOpen)
	{
    	std::this_thread::sleep_for(std::chrono::milliseconds(m_timeSlot));
	
	    std::lock_guard<std::mutex> lck (m_dataMutex);
	    
	    auto itMsg = m_messages.begin();
	    while(itMsg != m_messages.end())
	    {
	        if (itMsg->isExpired())
	        {
	            //pas besoin d'incrementer le pointeur on recupère le suivant
	            itMsg = m_messages.erase(itMsg);
	        }
	        else
	        {
	            if (itMsg->isReady())
	            {
	                if (m_transport != nullptr)
                    {
                        m_transport->send(itMsg->getMessage());
                    }
	            }
	            ++itMsg;
	        }
	    }        
	}
}

