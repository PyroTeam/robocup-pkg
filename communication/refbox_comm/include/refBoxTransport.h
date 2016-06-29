/**
 * \file 		refBoxTransport.h
 *
 * \brief		bibliothèque de gestion de la communication avec la referee Box
 * 				pour la competition robocup (Logistic League)
 * 				Version utilisant des ports spécifiques par équipe et le
 * 				chiffrage optionel des données
 *              (Sur la base du code llsf-fake-robot.cpp)
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2014-06-16
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef REFBOXTRANSPORT_H_
#define REFBOXTRANSPORT_H_

#include <config/yaml.h>

#include <protobuf_comm/peer.h>
#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>

#include <msgs/Team.pb.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <string>
#include <vector>

#include "dispatch.h"

using namespace protobuf_comm;
using namespace llsf_msgs;
using namespace fawkes;

class RefBoxComm;
typedef StaticDispatch<google::protobuf::Message, bool, std::function<bool(google::protobuf::Message&)>> MessageDispatcher;



class RefBoxTransport
{
private:
    std::shared_ptr<MessageDispatcher> m_msgDispatch;

public:
	RefBoxTransport();
	~RefBoxTransport();

	void init(std::string teamColor);
	void startTimer();
	void update();
	bool isExit();
	void handle_timer();
	
	void setReceiveDispatcher(std::shared_ptr<MessageDispatcher> disp)
	{
	    m_msgDispatch = disp;
	}

    void send(google::protobuf::Message &msg)
    {
        if (m_peer_team != NULL)
        {
            m_peer_team->send(msg);
        }
        else
        {
            std::cout << "m_peer_team_ not yet initialized!!" << std::endl;
        }
    }

protected:

	void signal_handler(const boost::system::error_code& error, int signum);
	void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
	void handle_send_error(std::string msg);
	
	void handle_message(boost::asio::ip::udp::endpoint &sender,
		       uint16_t component_id, uint16_t msg_type,
		       std::shared_ptr<google::protobuf::Message> msg);
	
	Team m_team_color;
	std::string m_team_name;
	unsigned long m_seq;
	ProtobufBroadcastPeer *m_peer_public;
	ProtobufBroadcastPeer *m_peer_team;
	bool m_crypto_setup;

	bool m_quit;

	std::string m_crypto_key;
	std::string m_cipher;

	boost::asio::io_service m_io_service;

    llsfrb::Configuration *m_config;

    boost::mutex m_dataMutex;
};


#endif /* REFBOXTRANSPORT_H_ */
