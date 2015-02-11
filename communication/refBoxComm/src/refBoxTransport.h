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
 * \copyright	RBQT, Polytech-Lille
 * \license
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

using namespace protobuf_comm;
using namespace llsf_msgs;
using namespace fawkes;

enum GamePhase
{
    PHASE_PRE_GAME,
    PHASE_SETUP,
    PHASE_EXPLORATION,
    PHASE_PRODUCTION,
    PHASE_POST_GAME
};

class refBoxTransport
{
public:
	refBoxTransport();
	~refBoxTransport();

	void init();
	void startTimer();
	void update();
	bool isExit();
	void handle_timer();

	void add_machine(std::string &name, std::string &type);
	void set_pose(double x, double y, double theta);
	llsf_msgs::GameState get_gameState();
	llsf_msgs::ExplorationInfo get_explorationInfo();
	GamePhase get_phase();
    Team get_teamColor()
    {
        return m_team_color_;
    }
        
protected:

	void signal_handler(const boost::system::error_code& error, int signum);
	void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
	void handle_send_error(std::string msg);
	void handle_message(boost::asio::ip::udp::endpoint &sender,
		       uint16_t component_id, uint16_t msg_type,
		       std::shared_ptr<google::protobuf::Message> msg);
	//void handle_timer(const boost::system::error_code& error);

	
	std::string m_name_;
	Team m_team_color_;
	std::string m_team_name_;
	unsigned int m_robot_number_;
	unsigned long m_seq_;
	ProtobufBroadcastPeer *m_peer_public_;
	ProtobufBroadcastPeer *m_peer_team_;
	bool m_crypto_setup_;

	bool m_quit;

	std::string m_crypto_key;
	std::string m_cipher;

	boost::asio::io_service m_io_service;

    llsfrb::Configuration *m_config_;


    llsf_msgs::MachineReport m_machineToReport;
    llsf_msgs::BeaconSignal m_beaconSignal;
    llsf_msgs::GameState m_gameState;
    llsf_msgs::ExplorationInfo m_explorationInfo;

    GamePhase m_gamePhase;

    boost::mutex m_dataMutex;
};


#endif /* REFBOXTRANSPORT_H_ */
