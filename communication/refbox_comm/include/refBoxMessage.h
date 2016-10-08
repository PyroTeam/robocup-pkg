/**
 * \file 		refBoxMessage.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */


#ifndef REFBOXMESSAGE_H_
#define REFBOXMESSAGE_H_

#include <list>
#include <memory>
#include <chrono>


//forward declaration
namespace google
{
    namespace protobuf
    {
        class Message;
    }
};


typedef google::protobuf::Message protoMsg;
typedef std::function<bool(protoMsg&)> RefBoxMsgCBType;  

class RefBoxMessage
{
public:
    enum Periodicity
    {
        PERIODIC,
        NON_PERIODIC
    };
    
    RefBoxMessage(std::shared_ptr<protoMsg> msg, Periodicity p = NON_PERIODIC, double period = 10.0, bool stopCond = false);
    virtual ~RefBoxMessage();
    
    void set(std::shared_ptr<protoMsg> msg, Periodicity p = NON_PERIODIC, double period = 10.0, bool stopCond = false);
    
    protoMsg &getMessage() const
    {
        return *m_message;
    }

 
    void setCallBack(RefBoxMsgCBType func)
    {
        m_callBackFunc = func;
    }
    
    bool isExpired();
    bool isReady();
    
private:
    std::shared_ptr<protoMsg> m_message;
    
    
    Periodicity m_periodicity;
    unsigned int m_period;
    bool m_stopCondition;
    
    bool m_isFirstSend;
    std::chrono::time_point<std::chrono::system_clock> m_lastSend;
    
    //callback
    RefBoxMsgCBType m_callBackFunc;
};


#endif /* REFBOXMESSAGE_H_ */
