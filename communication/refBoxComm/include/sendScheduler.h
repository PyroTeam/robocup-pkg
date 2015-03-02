/**
 * \file 		sendScheduler.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef SENDSCHEDULER_H_
#define SENDSCHEDULER_H_

#include <string>
#include <thread>
#include <memory>
#include <atomic>
#include <mutex>
#include "refBoxMessage.h"

//forward declaration
class refBoxTransport;

class SendScheduler
{
public:
    SendScheduler(std::shared_ptr<refBoxTransport> &refBoxTr);
    virtual ~SendScheduler();

	void close();
	void push(RefBoxMessage &message)
	{
		std::lock_guard<std::mutex> lck (m_dataMutex);
	    m_messages.push_back(message);
	}
	
	bool isOpened()
	{
	    return m_isOpen;
	}

private:
    unsigned int m_timeSlot;
	std::atomic<bool> m_isOpen;
   	
   	std::list<RefBoxMessage> m_messages;

	std::mutex m_dataMutex;
	std::unique_ptr<std::thread> m_t;
	
	std::shared_ptr<refBoxTransport> m_transport;
	
	void spawn() {
		m_t = std::unique_ptr<std::thread> (new std::thread(&SendScheduler::run, this));
	}
	void run();
};


#endif /* SENDSCHEDULER_H_ */
